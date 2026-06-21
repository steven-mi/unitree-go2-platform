"""Top-down floor plan from Go2 rolling-voxel lidar scans.

The Go2 streams a *local* voxel occupancy map (odom frame) that follows the
robot. We fuse those frames into a global 2D occupancy grid:

* **Evidence counting** — each cell keeps a small integer score. A scan that
  hits the cell as tall structure adds +1; a scan whose ray passes through the
  cell subtracts 1. A cell is a wall once its score reaches ``hits_to_confirm``
  and free once it drops to ``-clears_to_free``. Requiring several confirming
  hits rejects stray points, so noise and dynamic obstacles decay while real
  structure sharpens into thin, crisp walls.
* **Ray casting** — every scan clears the cells between the robot and each wall
  return, which carves out clean interiors and keeps walls one cell thick.
* **Height gate** — a cell only earns wall evidence when its tallest return
  reaches ``wall_height_min``, separating walls from flat floor returns.
"""

from __future__ import annotations

import base64
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

from config import cfg_float, cfg_int, load_config

_CFG = load_config()


@dataclass
class FloorPlanResult:
    width: int
    height: int
    origin_x: float
    origin_y: float
    resolution: float
    interior: np.ndarray
    outline: np.ndarray
    path: list[dict[str, float]]
    scan_count: int
    threshold: int


@dataclass
class FloorPlanBuilder:
    """Incremental evidence-count occupancy grid that refines into walls + interior."""

    resolution: float = field(default_factory=lambda: cfg_float(_CFG, "floorplan.resolution"))

    # Height band + wall gate (metres).
    z_min: float = field(default_factory=lambda: cfg_float(_CFG, "floorplan.z_min"))
    z_max: float = field(default_factory=lambda: cfg_float(_CFG, "floorplan.z_max"))
    wall_height_min: float = field(default_factory=lambda: cfg_float(_CFG, "floorplan.wall_height_min"))
    # A column earns wall evidence only with this many returns above wall height.
    # Real walls stack many voxels vertically; lone spurious returns do not.
    wall_min_points: int = field(default_factory=lambda: cfg_int(_CFG, "floorplan.wall_min_points"))

    # Evidence model (integer counts). A cell scores +1 per confirming wall hit
    # and -1 per ray clear; it is a wall at >= hits_to_confirm, free at
    # <= -clears_to_free. The score saturates at those bounds, so flipping a
    # settled cell takes (hits_to_confirm + clears_to_free) opposing scans.
    hits_to_confirm: int = field(default_factory=lambda: cfg_int(_CFG, "floorplan.hits_to_confirm"))
    clears_to_free: int = field(default_factory=lambda: cfg_int(_CFG, "floorplan.clears_to_free"))

    # Ray casting.
    max_ray_m: float = field(default_factory=lambda: cfg_float(_CFG, "floorplan.max_ray_m"))

    # Output cleanup.
    cleanup_radius: int = field(default_factory=lambda: cfg_int(_CFG, "floorplan.cleanup_radius"))
    min_wall_cells: int = field(default_factory=lambda: cfg_int(_CFG, "floorplan.min_wall_cells"))

    path_clearance_m: float = field(default_factory=lambda: cfg_float(_CFG, "floorplan.path_clearance_m"))
    margin_m: float = field(default_factory=lambda: cfg_float(_CFG, "floorplan.margin_m"))
    max_dim: int = field(default_factory=lambda: cfg_int(_CFG, "floorplan.max_dim"))

    origin_x: float = 0.0
    origin_y: float = 0.0
    width: int = 0
    height: int = 0

    score: np.ndarray | None = field(default=None, repr=False)
    path_pts: list[dict[str, float]] = field(default_factory=list)
    scan_count: int = 0
    _dirty: bool = field(default=True, repr=False)
    _cached: FloorPlanResult | None = field(default=None, repr=False)

    # ------------------------------------------------------------------ lifecycle

    def reset(self) -> None:
        self.score = None
        self.width = 0
        self.height = 0
        self.path_pts.clear()
        self.scan_count = 0
        self._dirty = True
        self._cached = None

    def append_path(self, x: float, y: float) -> None:
        if self.path_pts and self.path_pts[-1]["x"] == x and self.path_pts[-1]["y"] == y:
            return
        self.path_pts.append({"x": x, "y": y})
        self._dirty = True

    # --------------------------------------------------------------- persistence

    def save_state(self, path, *, revision: int) -> bool:
        """Persist the accumulated evidence grid so a session can resume mapping.

        Stores only the compact occupancy ``score`` grid plus its bounds and the
        walked path — not the raw lidar frames. The grid is exactly the builder
        state needed to keep fusing new scans, so restoring it continues mapping
        bit-for-bit. Returns ``False`` when there is nothing to save.
        """
        if self.score is None:
            return False
        if self.path_pts:
            path_arr = np.array([[p["x"], p["y"]] for p in self.path_pts], dtype=np.float32)
        else:
            path_arr = np.zeros((0, 2), dtype=np.float32)
        meta = np.array(
            [
                self.origin_x,
                self.origin_y,
                self.resolution,
                self.width,
                self.height,
                self.scan_count,
                revision,
            ],
            dtype=np.float64,
        )
        # Scores saturate at small integer bounds, so int16 is lossless and the
        # mostly-zero grid compresses to a tiny fraction of the raw lidar frames.
        np.savez_compressed(path, score=self.score.astype(np.int16), path=path_arr, meta=meta)
        return True

    @classmethod
    def from_state(cls, path) -> tuple["FloorPlanBuilder", int]:
        """Rebuild a builder from a saved evidence grid. Returns (builder, revision)."""
        with np.load(path) as data:
            meta = np.asarray(data["meta"], dtype=np.float64)
            score = np.asarray(data["score"], dtype=np.float32)
            path_arr = np.asarray(data["path"], dtype=np.float32)
        builder = cls(resolution=float(meta[2]))
        builder.score = score
        builder.origin_x = float(meta[0])
        builder.origin_y = float(meta[1])
        builder.width = int(meta[3])
        builder.height = int(meta[4])
        builder.scan_count = int(meta[5])
        builder.path_pts = [{"x": float(x), "y": float(y)} for x, y in path_arr]
        builder._dirty = True
        return builder, int(meta[6])

    # -------------------------------------------------------------------- ingest

    def ingest_points(
        self,
        points: np.ndarray,
        *,
        robot_x: float | None = None,
        robot_y: float | None = None,
        count_scan: bool = True,
    ) -> int:
        """Fuse one lidar voxel frame into the occupancy grid."""
        if points is None or len(points) == 0:
            return 0

        pts = np.asarray(points, dtype=np.float32)
        mask = np.isfinite(pts).all(axis=1) & (pts[:, 2] >= self.z_min) & (pts[:, 2] <= self.z_max)
        if not mask.any():
            return 0
        xs = pts[mask, 0]
        ys = pts[mask, 1]
        zs = pts[mask, 2]

        self._ensure_bounds(xs, ys, robot_x, robot_y)
        assert self.score is not None

        gi = np.floor((xs - self.origin_x) / self.resolution).astype(np.int32)
        gj = np.floor((ys - self.origin_y) / self.resolution).astype(np.int32)
        valid = (gi >= 0) & (gi < self.width) & (gj >= 0) & (gj < self.height)
        if not valid.any():
            return 0
        gi = gi[valid]
        gj = gj[valid]
        zs = zs[valid]

        # A cell is a wall return this frame when its column holds at least
        # wall_min_points returns above wall height. The floor is already clipped
        # by z_min, and a per-frame (not cumulative) test avoids body-height /
        # tilt drift inflating low cells. Counting tall returns per column (real
        # walls stack many voxels vertically, lone reflections do not) rejects
        # stray single points before they earn evidence — important because
        # hits_to_confirm can be 1, so one spurious frame would otherwise paint a
        # permanent wall.
        flat = gj.astype(np.int64) * self.width + gi.astype(np.int64)
        tall = zs >= self.wall_height_min
        uniq_tall, counts_tall = np.unique(flat[tall], return_counts=True)
        wall_cells = uniq_tall[counts_tall >= self.wall_min_points]
        wall_i = (wall_cells % self.width).astype(np.int32)
        wall_j = (wall_cells // self.width).astype(np.int32)

        if robot_x is not None and robot_y is not None:
            ri = int(math.floor((robot_x - self.origin_x) / self.resolution))
            rj = int(math.floor((robot_y - self.origin_y) / self.resolution))
            self._fuse_rays(ri, rj, wall_i, wall_j)
        else:
            # No pose: cannot ray cast, fall back to occupied evidence only.
            self._add_occupied(wall_j, wall_i)

        np.clip(self.score, -self.clears_to_free, self.hits_to_confirm, out=self.score)

        if count_scan:
            self.scan_count += 1
        self._dirty = True
        return int(valid.sum())

    def _add_occupied(self, gj: np.ndarray, gi: np.ndarray) -> None:
        assert self.score is not None
        np.add.at(self.score, (gj, gi), 1.0)

    def _fuse_rays(
        self,
        ri: int,
        rj: int,
        wall_i: np.ndarray,
        wall_j: np.ndarray,
    ) -> None:
        """Clear cells along each robot→wall ray, mark the wall cell occupied."""
        assert self.score is not None
        if wall_i.size == 0:
            return
        di = wall_i.astype(np.int32) - ri
        dj = wall_j.astype(np.int32) - rj
        steps = np.maximum(np.abs(di), np.abs(dj)).astype(np.int32)
        max_steps = max(1, int(round(self.max_ray_m / self.resolution)))
        steps = np.minimum(steps, max_steps)
        keep = steps > 0
        if not keep.any():
            self._add_occupied(wall_j, wall_i)
            return
        di_k = di[keep]
        dj_k = dj[keep]
        steps_k = steps[keep]
        n_steps = int(steps_k.max())

        # Protect this frame's wall returns: a ray toward a far wall must not clear
        # a nearer wall it grazes through (else thin walls erode away over time).
        wall_mask = np.zeros((self.height, self.width), dtype=bool)
        wall_mask[wall_j, wall_i] = True

        # Sample each ray at integer step fractions; clear interior cells (s < steps).
        s = np.arange(1, n_steps, dtype=np.float32)  # exclude origin (0) and endpoint
        if s.size:
            frac = s[None, :] / steps_k[:, None].astype(np.float32)
            valid = s[None, :] < steps_k[:, None]
            fi = np.round(ri + frac * di_k[:, None]).astype(np.int32)
            fj = np.round(rj + frac * dj_k[:, None]).astype(np.int32)
            in_bounds = (
                valid
                & (fi >= 0)
                & (fi < self.width)
                & (fj >= 0)
                & (fj < self.height)
            )
            fjb = fj[in_bounds]
            fib = fi[in_bounds]
            free_sel = ~wall_mask[fjb, fib]
            np.add.at(self.score, (fjb[free_sel], fib[free_sel]), -1.0)

        # Wall endpoints get occupied evidence (net positive — never cleared).
        self._add_occupied(wall_j, wall_i)

    # --------------------------------------------------------------------- build

    def build(
        self,
        *,
        crop: bool = True,
        robot_x: float | None = None,
        robot_y: float | None = None,
    ) -> FloorPlanResult:
        use_cache = robot_x is None and robot_y is None
        if use_cache and not self._dirty and self._cached is not None:
            return self._cached
        if not self._has_wall_data():
            raise ValueError("No lidar points in wall-height band")
        assert self.score is not None
        result = _finalize_grid(
            self.score,
            self.path_pts,
            self.origin_x,
            self.origin_y,
            self.resolution,
            self.width,
            self.height,
            hits_to_confirm=self.hits_to_confirm,
            clears_to_free=self.clears_to_free,
            wall_height_min=self.wall_height_min,
            cleanup_radius=self.cleanup_radius,
            min_wall_cells=self.min_wall_cells,
            path_clearance_m=self.path_clearance_m,
            robot_x=robot_x,
            robot_y=robot_y,
            crop=crop,
            scan_count=self.scan_count,
        )
        if use_cache:
            self._cached = result
            self._dirty = False
        return result

    def _has_wall_data(self) -> bool:
        if self.score is None:
            return False
        return bool(np.any(self.score >= self.hits_to_confirm))

    # ----------------------------------------------------------------- internals

    def _ensure_bounds(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        robot_x: float | None,
        robot_y: float | None,
    ) -> None:
        res = self.resolution
        margin = self.margin_m
        candidates_x = [float(xs.min()), float(xs.max())]
        candidates_y = [float(ys.min()), float(ys.max())]
        if robot_x is not None:
            candidates_x.append(robot_x)
        if robot_y is not None:
            candidates_y.append(robot_y)
        xmin = min(candidates_x) - margin
        ymin = min(candidates_y) - margin
        xmax = max(candidates_x) + margin
        ymax = max(candidates_y) + margin

        if self.score is None:
            self.origin_x = xmin
            self.origin_y = ymin
            self.width = max(1, int(math.ceil((xmax - xmin) / res)))
            self.height = max(1, int(math.ceil((ymax - ymin) / res)))
            self._apply_max_dim()
            self.score = np.zeros((self.height, self.width), dtype=np.float32)
            return

        cur_x1 = self.origin_x + self.width * res
        cur_y1 = self.origin_y + self.height * res
        need_x0 = min(self.origin_x, xmin)
        need_y0 = min(self.origin_y, ymin)
        need_x1 = max(cur_x1, xmax)
        need_y1 = max(cur_y1, ymax)
        if need_x0 == self.origin_x and need_y0 == self.origin_y and need_x1 == cur_x1 and need_y1 == cur_y1:
            return

        new_w = max(1, int(math.ceil((need_x1 - need_x0) / res)))
        new_h = max(1, int(math.ceil((need_y1 - need_y0) / res)))
        off_i = int(round((self.origin_x - need_x0) / res))
        off_j = int(round((self.origin_y - need_y0) / res))

        new_score = np.zeros((new_h, new_w), dtype=np.float32)
        assert self.score is not None
        new_score[off_j : off_j + self.height, off_i : off_i + self.width] = self.score

        self.score = new_score
        self.origin_x = need_x0
        self.origin_y = need_y0
        self.width = new_w
        self.height = new_h
        self._apply_max_dim()

    def _apply_max_dim(self) -> None:
        if self.score is None or (self.width <= self.max_dim and self.height <= self.max_dim):
            return
        scale = max(self.width / self.max_dim, self.height / self.max_dim)
        self._downsample_grid(int(math.ceil(scale)))

    def _downsample_grid(self, scale: float) -> None:
        if self.score is None:
            return
        factor = int(math.ceil(scale))
        if factor <= 1:
            return
        h, w = self.score.shape
        new_h = max(1, int(math.ceil(h / factor)))
        new_w = max(1, int(math.ceil(w / factor)))
        pad_h = new_h * factor - h
        pad_w = new_w * factor - w

        lo = np.pad(self.score, ((0, pad_h), (0, pad_w)), constant_values=0.0)
        self.score = lo.reshape(new_h, factor, new_w, factor).max(axis=(1, 3)).astype(np.float32)
        self.resolution *= factor
        self.width = new_w
        self.height = new_h


# ---------------------------------------------------------------------- morphology


def _cv_mask(mask: np.ndarray, op: int, radius: int) -> np.ndarray:
    if radius <= 0:
        return mask
    k = radius * 2 + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    return cv2.morphologyEx(mask.astype(np.uint8), op, kernel).astype(bool)


def morph_close(mask: np.ndarray, radius: int) -> np.ndarray:
    return _cv_mask(mask, cv2.MORPH_CLOSE, radius)


def morph_open(mask: np.ndarray, radius: int) -> np.ndarray:
    return _cv_mask(mask, cv2.MORPH_OPEN, radius)


def morph_dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    if radius <= 0:
        return mask
    k = radius * 2 + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    return cv2.dilate(mask.astype(np.uint8), kernel).astype(bool)


def _remove_small_components(mask: np.ndarray, min_cells: int) -> np.ndarray:
    """Drop speckle blobs while preserving thin 1-cell-wide walls (no erosion)."""
    if min_cells <= 1 or not mask.any():
        return mask
    n, _labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8
    )
    if n <= 1:
        return mask
    keep = np.zeros(n, dtype=bool)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= min_cells
    return keep[_labels]


# ----------------------------------------------------------------------- fill/crop


def _seed_flood(
    reachable: np.ndarray,
    queue: deque[tuple[int, int]],
    free: np.ndarray,
    x0: float,
    y0: float,
    resolution: float,
    x: float,
    y: float,
) -> None:
    h, w = free.shape
    gi = int((x - x0) / resolution)
    gj = int((y - y0) / resolution)
    if 0 <= gi < w and 0 <= gj < h and free[gj, gi] and not reachable[gj, gi]:
        reachable[gj, gi] = True
        queue.append((gj, gi))


def _flood_reachable(
    free: np.ndarray,
    path_pts: list[dict[str, float]],
    x0: float,
    y0: float,
    resolution: float,
    robot_x: float | None = None,
    robot_y: float | None = None,
) -> np.ndarray:
    h, w = free.shape
    reachable = np.zeros((h, w), dtype=bool)
    queue: deque[tuple[int, int]] = deque()
    for pt in path_pts:
        _seed_flood(reachable, queue, free, x0, y0, resolution, pt["x"], pt["y"])
    if robot_x is not None and robot_y is not None:
        _seed_flood(reachable, queue, free, x0, y0, resolution, robot_x, robot_y)
    while queue:
        j, i = queue.popleft()
        for dj, di in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nj, ni = j + dj, i + di
            if 0 <= nj < h and 0 <= ni < w and free[nj, ni] and not reachable[nj, ni]:
                reachable[nj, ni] = True
                queue.append((nj, ni))
    return reachable


def _flood_exterior(free_mask: np.ndarray) -> np.ndarray:
    h, w = free_mask.shape
    outside = np.zeros((h, w), dtype=bool)
    queue: deque[tuple[int, int]] = deque()
    for i in range(w):
        if free_mask[0, i]:
            queue.append((0, i))
            outside[0, i] = True
        if free_mask[h - 1, i]:
            queue.append((h - 1, i))
            outside[h - 1, i] = True
    for j in range(h):
        if free_mask[j, 0]:
            queue.append((j, 0))
            outside[j, 0] = True
        if free_mask[j, w - 1]:
            queue.append((j, w - 1))
            outside[j, w - 1] = True
    while queue:
        j, i = queue.popleft()
        for dj, di in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nj, ni = j + dj, i + di
            if 0 <= nj < h and 0 <= ni < w and free_mask[nj, ni] and not outside[nj, ni]:
                outside[nj, ni] = True
                queue.append((nj, ni))
    return outside


def _fill_holes(mask: np.ndarray) -> np.ndarray:
    holes = (~mask) & (~_flood_exterior(~mask))
    return mask | holes


def _crop_plan(
    interior: np.ndarray,
    outline: np.ndarray,
    x0: float,
    y0: float,
    resolution: float,
    pad_m: float = 0.35,
) -> tuple[np.ndarray, np.ndarray, float, float, int, int]:
    mask = interior | outline
    if not mask.any():
        return interior, outline, x0, y0, interior.shape[1], interior.shape[0]

    rows = np.where(mask.any(axis=1))[0]
    cols = np.where(mask.any(axis=0))[0]
    pad = max(1, int(round(pad_m / resolution)))
    r0 = max(0, int(rows[0]) - pad)
    r1 = min(mask.shape[0] - 1, int(rows[-1]) + pad)
    c0 = max(0, int(cols[0]) - pad)
    c1 = min(mask.shape[1] - 1, int(cols[-1]) + pad)

    interior = interior[r0 : r1 + 1, c0 : c1 + 1]
    outline = outline[r0 : r1 + 1, c0 : c1 + 1]
    return (
        interior,
        outline,
        x0 + c0 * resolution,
        y0 + r0 * resolution,
        c1 - c0 + 1,
        r1 - r0 + 1,
    )


def _corridor_mask(
    path_pts: list[dict[str, float]],
    x0: float,
    y0: float,
    resolution: float,
    height: int,
    width: int,
    radius_m: float,
    robot_x: float | None = None,
    robot_y: float | None = None,
) -> np.ndarray:
    """Disc around the walked path (+ live pose); clears wall smear where the dog drove."""
    mask = np.zeros((height, width), dtype=np.uint8)
    seeds = [(p["x"], p["y"]) for p in path_pts]
    if robot_x is not None and robot_y is not None:
        seeds.append((robot_x, robot_y))
    for x, y in seeds:
        gi = int((x - x0) / resolution)
        gj = int((y - y0) / resolution)
        if 0 <= gi < width and 0 <= gj < height:
            mask[gj, gi] = 1
    if not mask.any():
        return mask.astype(bool)
    return morph_dilate(mask.astype(bool), max(1, int(round(radius_m / resolution))))


def _finalize_grid(
    score: np.ndarray,
    path_pts: list[dict[str, float]],
    x0: float,
    y0: float,
    resolution: float,
    width: int,
    height: int,
    *,
    hits_to_confirm: int,
    clears_to_free: int,
    wall_height_min: float,
    cleanup_radius: int,
    min_wall_cells: int,
    path_clearance_m: float,
    robot_x: float | None,
    robot_y: float | None,
    crop: bool,
    scan_count: int,
) -> FloorPlanResult:
    """Turn the evidence-count grid into wall outlines + explored free interior."""
    # Occupied evidence is only ever added to per-frame wall columns, so the
    # score sign alone classifies walls vs. free without a height re-check here.
    walls = score >= hits_to_confirm
    free = score <= -clears_to_free

    # Carve out wall smear along the path the dog actually walked through.
    if path_clearance_m > 0 and (path_pts or (robot_x is not None and robot_y is not None)):
        corridor = _corridor_mask(
            path_pts, x0, y0, resolution, height, width,
            radius_m=path_clearance_m, robot_x=robot_x, robot_y=robot_y,
        )
        walls = walls & ~corridor
        free = free | corridor

    # Bridge gaps, then drop isolated speckle by area (keeps walls thin).
    walls = morph_close(walls, cleanup_radius)
    walls = _remove_small_components(walls, min_cells=min_wall_cells)
    free = free & ~walls

    reachable = _flood_reachable(free, path_pts, x0, y0, resolution, robot_x=robot_x, robot_y=robot_y)
    if not reachable.any():
        reachable = free
    interior = _fill_holes(reachable) & ~walls
    interior = morph_open(interior, cleanup_radius // 2) if cleanup_radius > 1 else interior

    walls = walls & ~interior

    if crop:
        interior, walls, x0, y0, width, height = _crop_plan(interior, walls, x0, y0, resolution)

    return FloorPlanResult(
        width=width,
        height=height,
        origin_x=round(x0, 4),
        origin_y=round(y0, 4),
        resolution=round(resolution, 4),
        interior=interior,
        outline=walls,
        path=list(path_pts),
        scan_count=scan_count,
        threshold=int(math.ceil(wall_height_min * 100)),
    )


def floor_plan_to_api(plan: FloorPlanResult, *, upto_t: float | None = None) -> dict[str, Any]:
    zones = np.zeros((plan.height, plan.width), dtype=np.uint8)
    zones[plan.interior] = 1
    walls = plan.outline.astype(np.uint8)
    out: dict[str, Any] = {
        "width": plan.width,
        "height": plan.height,
        "origin_x": plan.origin_x,
        "origin_y": plan.origin_y,
        "resolution": plan.resolution,
        "scan_count": plan.scan_count,
        "threshold": plan.threshold,
        "zone_count": int(zones.max()),
        "map_rotation": 0.0,
        "zones_b64": base64.b64encode(zones.tobytes()).decode("ascii"),
        "walls_b64": base64.b64encode(walls.tobytes()).decode("ascii"),
        "path": plan.path,
    }
    if upto_t is not None:
        out["upto_t"] = round(upto_t, 3)
    return out
