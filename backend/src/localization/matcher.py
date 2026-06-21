"""2D lidar scan matching against a saved floor-plan wall grid.

The match is a rigid SE(2) transform that maps the robot's *odom* frame onto the
saved *map* frame:

    p_map = R(dyaw) · p_odom + (tx, ty)

Candidate poses rotate the scan **about the robot's current odom position** (not
the odom origin), so the translation search is a pure position offset that stays
valid no matter how far odom has drifted from where the map was built. Scoring is
a chamfer / distance-transform match: each scan point scores by its metric
distance to the nearest wall, giving a smooth cost surface instead of a binary
overlap count.
"""

from __future__ import annotations

import base64
import math
from typing import Any

import cv2
import numpy as np

Z_MIN = 0.15
Z_MAX = 1.35
MAX_SCAN_POINTS = 1200
COARSE_SCAN_POINTS = 500  # subsample for the coarse sweep; full set for the refine

# Chamfer scoring: a scan point `d` metres from the nearest wall scores
# exp(-d / SCORE_SIGMA_M). On a wall it scores 1.0; one sigma away ~0.37. The
# match score is the mean over every scan point (off-map points score 0), so
# transforms that fling the scan off the explored area are penalised.
SCORE_SIGMA_M = 0.18
MIN_SCORE = 0.45
GOOD_SCORE = 0.72


def decode_grids(floorplan: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, float, float, float, int, int]:
    width = int(floorplan["width"])
    height = int(floorplan["height"])
    origin_x = float(floorplan["origin_x"])
    origin_y = float(floorplan["origin_y"])
    resolution = float(floorplan["resolution"])
    size = width * height

    walls_raw = base64.b64decode(floorplan["walls_b64"])
    zones_raw = base64.b64decode(floorplan.get("zones_b64", ""))
    walls = np.frombuffer(walls_raw, dtype=np.uint8).reshape(height, width)
    zones = (
        np.frombuffer(zones_raw, dtype=np.uint8).reshape(height, width)
        if len(zones_raw) >= size
        else np.zeros((height, width), dtype=np.uint8)
    )
    return walls, zones, origin_x, origin_y, resolution, width, height


def lidar_points_2d(points: np.ndarray, *, z_min: float = Z_MIN, z_max: float = Z_MAX) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 2:
        return np.empty((0, 2), dtype=np.float32)
    mask = np.isfinite(pts).all(axis=1)
    if pts.shape[1] >= 3:
        mask &= (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
    pts = pts[mask, :2]
    if len(pts) > MAX_SCAN_POINTS:
        idx = np.linspace(0, len(pts) - 1, MAX_SCAN_POINTS, dtype=int)
        pts = pts[idx]
    return pts.astype(np.float32)


def _wall_score_grid(walls: np.ndarray, resolution: float) -> np.ndarray:
    """Chamfer score grid: exp(-distance_to_nearest_wall / sigma), per cell."""
    occupied = (walls > 0).astype(np.uint8)
    free = np.where(occupied > 0, 0, 255).astype(np.uint8)
    dist_px = cv2.distanceTransform(free, cv2.DIST_L2, 3)
    dist_m = dist_px * float(resolution)
    return np.exp(-dist_m / SCORE_SIGMA_M).astype(np.float32)


def _world_to_grid(
    xs: np.ndarray,
    ys: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    gi = np.floor((xs - origin_x) / resolution).astype(np.int32)
    gj = np.floor((ys - origin_y) / resolution).astype(np.int32)
    valid = (gi >= 0) & (gi < width) & (gj >= 0) & (gj < height)
    return gi, gj, valid


def _score_points(
    xy: np.ndarray,
    score_grid: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
) -> float:
    """Mean chamfer score over all points; off-map points contribute 0."""
    n = len(xy)
    if n == 0:
        return 0.0
    gi, gj, valid = _world_to_grid(xy[:, 0], xy[:, 1], origin_x, origin_y, resolution, width, height)
    if not valid.any():
        return 0.0
    return float(np.sum(score_grid[gj[valid], gi[valid]])) / n


def _rotate(scan_centered: np.ndarray, yaw: float) -> np.ndarray:
    """Rotate robot-centred scan points about the origin (the robot) by ``yaw``."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack(
        [
            c * scan_centered[:, 0] - s * scan_centered[:, 1],
            s * scan_centered[:, 0] + c * scan_centered[:, 1],
        ]
    ).astype(np.float32)


def _normalize_angle(rad: float) -> float:
    return (float(rad) + math.pi) % (2 * math.pi) - math.pi


def pose_in_explored(
    x: float,
    y: float,
    floorplan: dict[str, Any],
) -> bool:
    """True when a world pose lies on the scanned portion of the floor plan."""
    walls, zones, origin_x, origin_y, resolution, width, height = decode_grids(floorplan)
    gi = int(math.floor((x - origin_x) / resolution))
    gj = int(math.floor((y - origin_y) / resolution))
    if not (0 <= gi < width and 0 <= gj < height):
        return False
    return bool(zones[gj, gi] > 0) or bool(walls[gj, gi] > 0)


def _search_window(
    hint_x: float,
    hint_y: float,
    hint_dyaw: float | None,
    *,
    xy_radius: float,
    yaw_range: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    xy_step = 0.2 if xy_radius > 2.5 else 0.15
    xs = np.arange(hint_x - xy_radius, hint_x + xy_radius + 1e-6, xy_step)
    ys = np.arange(hint_y - xy_radius, hint_y + xy_radius + 1e-6, xy_step)
    if hint_dyaw is None:
        yaws = np.deg2rad(np.arange(0, 360, 15))
    else:
        yaws = np.arange(hint_dyaw - yaw_range, hint_dyaw + yaw_range + 1e-6, math.radians(12))
    return xs, ys, yaws


def _score_grid_of_yaw(
    scan_centered: np.ndarray,
    yaw: float,
    xs: np.ndarray,
    ys: np.ndarray,
    score_grid: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
) -> tuple[float, float, float]:
    """Best (score, cx, cy) for one yaw — rotate once, then sweep translations."""
    rot = _rotate(scan_centered, yaw)
    best = (-1.0, 0.0, 0.0)
    for cx in xs:
        for cy in ys:
            mapped = rot + np.array([cx, cy], dtype=np.float32)
            score = _score_points(mapped, score_grid, origin_x, origin_y, resolution, width, height)
            if score > best[0]:
                best = (score, float(cx), float(cy))
    return best


def _search_best_transform(
    scan_centered: np.ndarray,
    score_grid: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    *,
    hint_x: float,
    hint_y: float,
    hint_dyaw: float | None,
    search_radius_m: float,
    yaw_search_rad: float,
) -> tuple[float, tuple[float, float, float]]:
    xs, ys, yaws = _search_window(
        hint_x,
        hint_y,
        hint_dyaw,
        xy_radius=search_radius_m,
        yaw_range=yaw_search_rad,
    )

    coarse = scan_centered
    if len(scan_centered) > COARSE_SCAN_POINTS:
        idx = np.linspace(0, len(scan_centered) - 1, COARSE_SCAN_POINTS, dtype=int)
        coarse = scan_centered[idx]

    best_score = -1.0
    best_transform = (hint_x, hint_y, hint_dyaw or 0.0)
    for yaw in yaws:
        score, cx, cy = _score_grid_of_yaw(
            coarse, float(yaw), xs, ys, score_grid, origin_x, origin_y, resolution, width, height
        )
        if score > best_score:
            best_score = score
            best_transform = (cx, cy, float(yaw))

    tx, ty, tyaw = best_transform
    fine_xs = np.arange(tx - 0.3, tx + 0.31, 0.08)
    fine_ys = np.arange(ty - 0.3, ty + 0.31, 0.08)
    fine_yaws = np.arange(tyaw - math.radians(15), tyaw + math.radians(16), math.radians(4))
    best_score = -1.0
    for yaw in fine_yaws:
        score, cx, cy = _score_grid_of_yaw(
            scan_centered, float(yaw), fine_xs, fine_ys, score_grid, origin_x, origin_y, resolution, width, height
        )
        if score > best_score:
            best_score = score
            best_transform = (cx, cy, float(yaw))

    return best_score, best_transform


def _result_from_transform(
    best_score: float,
    transform: tuple[float, float, float],
    scan_centered: np.ndarray,
    robot: tuple[float, float, float],
) -> dict[str, Any]:
    cx, cy, dyaw = transform
    dyaw = _normalize_angle(dyaw)
    ok = best_score >= MIN_SCORE
    confidence = min(1.0, max(0.0, (best_score - MIN_SCORE) / max(GOOD_SCORE - MIN_SCORE, 1e-6)))

    rx, ry, ryaw = robot
    myaw = _normalize_angle(ryaw + dyaw)
    c, s = math.cos(dyaw), math.sin(dyaw)
    tx = cx - (c * rx - s * ry)
    ty = cy - (s * rx + c * ry)

    return {
        "ok": ok,
        "map_pose": {"x": round(float(cx), 4), "y": round(float(cy), 4), "yaw": round(myaw, 4)},
        "map_alignment": {
            "tx": round(float(tx), 4),
            "ty": round(float(ty), 4),
            "dyaw": round(dyaw, 4),
        },
        "score": round(best_score, 4),
        "confidence": round(confidence, 3),
        "point_count": int(len(scan_centered)),
        "reason": None if ok else "low_match_score",
    }


def estimate_map_pose(
    scan_xy: np.ndarray,
    floorplan: dict[str, Any],
    live_pose: dict[str, float],
    *,
    hint_x: float | None = None,
    hint_y: float | None = None,
    hint_dyaw: float | None = None,
    search_radius_m: float = 5.0,
    yaw_search_rad: float = math.pi,
) -> dict[str, Any]:
    """Find the robot's map pose that best explains the live lidar scan.

    ``hint_x``/``hint_y`` are the expected robot position *in the map frame*;
    ``hint_dyaw`` is the expected odom→map yaw offset. Leave them ``None`` for a
    cold (global) search.
    """
    scan_xy = lidar_points_2d(scan_xy)
    if len(scan_xy) < 40:
        return {"ok": False, "reason": "not_enough_lidar_points", "point_count": int(len(scan_xy))}

    walls, zones, origin_x, origin_y, resolution, width, height = decode_grids(floorplan)
    if not walls.any() and not zones.any():
        return {"ok": False, "reason": "empty_map"}

    score_grid = _wall_score_grid(walls, resolution)
    explored = (zones > 0) | (walls > 0)
    if explored.any():
        ys, xs = np.where(explored)
        map_cx = origin_x + (xs.mean() + 0.5) * resolution
        map_cy = origin_y + (ys.mean() + 0.5) * resolution
    else:
        map_cx = origin_x + width * resolution * 0.5
        map_cy = origin_y + height * resolution * 0.5

    rx = float(live_pose["x"])
    ry = float(live_pose["y"])
    ryaw = float(live_pose.get("yaw") or 0.0)
    robot = (rx, ry, ryaw)
    scan_centered = (scan_xy - np.array([rx, ry], dtype=np.float32)).astype(np.float32)

    seed_x = map_cx if hint_x is None else float(hint_x)
    seed_y = map_cy if hint_y is None else float(hint_y)

    best_score, best_transform = _search_best_transform(
        scan_centered,
        score_grid,
        origin_x,
        origin_y,
        resolution,
        width,
        height,
        hint_x=seed_x,
        hint_y=seed_y,
        hint_dyaw=hint_dyaw,
        search_radius_m=search_radius_m,
        yaw_search_rad=yaw_search_rad,
    )

    result = _result_from_transform(best_score, best_transform, scan_centered, robot)
    mx = result["map_pose"]["x"]
    my = result["map_pose"]["y"]
    used_wide_hint = search_radius_m >= 4.5 and yaw_search_rad >= math.pi - 1e-6

    if result["ok"] and not pose_in_explored(mx, my, floorplan) and not used_wide_hint:
        wide_score, wide_transform = _search_best_transform(
            scan_centered,
            score_grid,
            origin_x,
            origin_y,
            resolution,
            width,
            height,
            hint_x=map_cx,
            hint_y=map_cy,
            hint_dyaw=None,
            search_radius_m=5.0,
            yaw_search_rad=math.pi,
        )
        wide_result = _result_from_transform(wide_score, wide_transform, scan_centered, robot)
        if wide_result["ok"] and pose_in_explored(
            wide_result["map_pose"]["x"],
            wide_result["map_pose"]["y"],
            floorplan,
        ):
            return wide_result

    return result
