"""A* path planning on floor-plan occupancy grids."""

from __future__ import annotations

import heapq
import math
from collections import deque
from typing import Any, Callable

import cv2
import numpy as np

from config import cfg_float, cfg_int, configured_planner_clearance, load_config
from localization.matcher import decode_grids

_CFG = load_config()

# Search.
MAX_SNAP_RADIUS = cfg_int(_CFG, "planner.max_snap_radius")

# Live-obstacle detection — isolate *new* obstacles standing in explored free
# space (not already part of the saved walls) from a live lidar voxel frame.
OBSTACLE_Z_MIN = cfg_float(_CFG, "obstacle.z_min")
OBSTACLE_Z_MAX = cfg_float(_CFG, "obstacle.z_max")
OBSTACLE_DILATE_CELLS = cfg_int(_CFG, "obstacle.dilate_cells")
OBSTACLE_WALL_GUARD_CELLS = cfg_int(_CFG, "obstacle.wall_guard_cells")
OBSTACLE_MIN_CELLS = cfg_int(_CFG, "obstacle.min_cells")
OBSTACLE_SELF_CLEAR_M = cfg_float(_CFG, "obstacle.self_clear_m")
# Forward field-of-view: only returns within this half-angle of the heading count
# as obstacles, so something off to the side or behind the dog never reroutes it.
# 180 deg disables the gate (detect all around).
OBSTACLE_FRONT_HALF_ANGLE_RAD = math.radians(cfg_float(_CFG, "obstacle.front_half_angle_deg"))

_NEIGHBORS = (
    (1, 0, 1.0),
    (0, 1, 1.0),
    (-1, 0, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)),
    (-1, -1, math.sqrt(2)),
)


def _world_to_cell(x: float, y: float, origin_x: float, origin_y: float, resolution: float) -> tuple[int, int]:
    gi = int(math.floor((x - origin_x) / resolution))
    gj = int(math.floor((y - origin_y) / resolution))
    return gi, gj


def _cell_to_world(gi: int, gj: int, origin_x: float, origin_y: float, resolution: float) -> tuple[float, float]:
    x = origin_x + (gi + 0.5) * resolution
    y = origin_y + (gj + 0.5) * resolution
    return x, y


def _navigable_grid(
    walls: np.ndarray,
    zones: np.ndarray,
    resolution: float,
    *,
    robot_radius_m: float,
) -> np.ndarray:
    radius_cells = max(1, int(math.ceil(robot_radius_m / resolution)))
    k = radius_cells * 2 + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    blocked = cv2.dilate(walls.astype(np.uint8), kernel) > 0
    if zones.any():
        return (zones > 0) & ~blocked
    return ~blocked


def _remove_small_components_u8(mask: np.ndarray, min_cells: int) -> np.ndarray:
    if min_cells <= 1 or not mask.any():
        return mask
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = np.zeros_like(mask)
    for i in range(1, count):
        if stats[i, cv2.CC_STAT_AREA] >= min_cells:
            out[labels == i] = 1
    return out


def lidar_obstacle_mask(
    floorplan: dict[str, Any],
    points: np.ndarray | None,
    *,
    tx: float = 0.0,
    ty: float = 0.0,
    dyaw: float = 0.0,
    robot_xy: tuple[float, float] | None = None,
    robot_pose: tuple[float, float, float] | None = None,
) -> np.ndarray | None:
    """Project a live lidar frame into the floor-plan grid and isolate new obstacles.

    Returns a bool grid (H×W) of cells blocked by something that is not already in
    the saved walls — i.e. obstacles standing in explored free space. Points are
    mapped odom→map with the localization transform ``p_map = R(dyaw)·p_odom +
    (tx, ty)`` so they line up with poses/routes, which are aligned the same way.

    When ``robot_pose`` (odom-frame ``x, y, yaw``) is given and
    ``obstacle.front_half_angle_deg`` is below 180, returns outside the forward
    field-of-view cone are dropped so only obstacles *ahead* of the dog count.
    Returns ``None`` when nothing qualifies.
    """
    if points is None or len(points) == 0:
        return None
    walls, zones, origin_x, origin_y, resolution, width, height = decode_grids(floorplan)

    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3:
        return None
    z = pts[:, 2]
    keep = np.isfinite(pts).all(axis=1) & (z >= OBSTACLE_Z_MIN) & (z <= OBSTACLE_Z_MAX)

    # Forward field-of-view gate: keep only returns within the heading cone. The
    # robot pose and the raw lidar points share the odom frame, so the body-frame
    # bearing is computed before the odom→map transform below.
    if robot_pose is not None and OBSTACLE_FRONT_HALF_ANGLE_RAD < math.pi:
        rx, ry, ryaw = robot_pose
        dx = pts[:, 0].astype(np.float64) - rx
        dy = pts[:, 1].astype(np.float64) - ry
        c, s = math.cos(ryaw), math.sin(ryaw)
        bx = c * dx + s * dy
        by = -s * dx + c * dy
        bearing = np.abs(np.arctan2(by, bx))
        keep &= bearing <= OBSTACLE_FRONT_HALF_ANGLE_RAD

    if not keep.any():
        return None

    c, s = math.cos(dyaw), math.sin(dyaw)
    ox = pts[keep, 0]
    oy = pts[keep, 1]
    mx = c * ox - s * oy + tx
    my = s * ox + c * oy + ty
    gi = np.floor((mx - origin_x) / resolution).astype(np.int32)
    gj = np.floor((my - origin_y) / resolution).astype(np.int32)
    inb = (gi >= 0) & (gi < width) & (gj >= 0) & (gj < height)
    if not inb.any():
        return None
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[gj[inb], gi[inb]] = 1

    if OBSTACLE_DILATE_CELLS > 0:
        k = OBSTACLE_DILATE_CELLS * 2 + 1
        mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))

    # Drop returns that just re-hit known walls (localization slop near walls).
    if walls.any() and OBSTACLE_WALL_GUARD_CELLS > 0:
        wk = OBSTACLE_WALL_GUARD_CELLS * 2 + 1
        guard = cv2.dilate(walls.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (wk, wk)))
        mask[guard > 0] = 0

    # Only obstacles inside explored free space matter for routing.
    if zones.any():
        mask[zones == 0] = 0

    # The robot is never an obstacle to itself.
    if robot_xy is not None:
        rgi = int(math.floor((robot_xy[0] - origin_x) / resolution))
        rgj = int(math.floor((robot_xy[1] - origin_y) / resolution))
        radius = max(1, int(math.ceil(OBSTACLE_SELF_CLEAR_M / resolution)))
        cv2.circle(mask, (rgi, rgj), radius, 0, -1)

    mask = _remove_small_components_u8(mask, OBSTACLE_MIN_CELLS)
    if not mask.any():
        return None
    return mask.astype(bool)


def make_known_wall_filter(
    floorplan: dict[str, Any],
    *,
    tx: float = 0.0,
    ty: float = 0.0,
    dyaw: float = 0.0,
    guard_cells: int = OBSTACLE_WALL_GUARD_CELLS,
) -> Callable[[np.ndarray], np.ndarray] | None:
    """Build a classifier that flags lidar returns landing on a known map wall.

    Returns a callable mapping an ``(N, 3)`` odom-frame point array to a bool mask
    (length ``N``) that is ``True`` wherever the point falls within ``guard_cells``
    of a saved wall — i.e. something the map already contains and the planner
    already routed clear of by ``robot_radius_m``. The driving safety cone uses
    this to ignore mapped walls and only halt for genuinely new/unmapped
    obstacles (people, furniture, etc.).

    Points are mapped odom→map with the same transform the planner/localizer use
    (``p_map = R(dyaw)·p_odom + (tx, ty)``). The ``guard_cells`` dilation absorbs
    localization slop near walls. Returns ``None`` when the plan has no walls.
    """
    walls, _zones, origin_x, origin_y, resolution, width, height = decode_grids(floorplan)
    if not walls.any():
        return None
    if guard_cells > 0:
        wk = guard_cells * 2 + 1
        guard = (
            cv2.dilate(
                walls.astype(np.uint8),
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (wk, wk)),
            )
            > 0
        )
    else:
        guard = walls.astype(bool)
    c, s = math.cos(dyaw), math.sin(dyaw)

    def is_known_wall(points: np.ndarray) -> np.ndarray:
        pts = np.asarray(points, dtype=np.float64)
        n = len(pts)
        if pts.ndim != 2 or pts.shape[1] < 2 or n == 0:
            return np.zeros(n, dtype=bool)
        mx = c * pts[:, 0] - s * pts[:, 1] + tx
        my = s * pts[:, 0] + c * pts[:, 1] + ty
        gi = np.floor((mx - origin_x) / resolution).astype(np.int32)
        gj = np.floor((my - origin_y) / resolution).astype(np.int32)
        out = np.zeros(n, dtype=bool)
        inb = (gi >= 0) & (gi < width) & (gj >= 0) & (gj < height)
        out[inb] = guard[gj[inb], gi[inb]]
        return out

    return is_known_wall


def _component_cells(free: np.ndarray, seed: tuple[int, int]) -> set[tuple[int, int]]:
    width = free.shape[1]
    height = free.shape[0]
    si, sj = seed
    if not _in_bounds(si, sj, width, height) or not free[sj, si]:
        return set()
    seen: set[tuple[int, int]] = set()
    queue: deque[tuple[int, int]] = deque([seed])
    while queue:
        ci, cj = queue.popleft()
        if (ci, cj) in seen:
            continue
        seen.add((ci, cj))
        for di, dj, _ in _NEIGHBORS:
            ni, nj = ci + di, cj + dj
            if _in_bounds(ni, nj, width, height) and free[nj, ni] and (ni, nj) not in seen:
                queue.append((ni, nj))
    return seen


def _nearest_free(
    free: np.ndarray,
    gi: int,
    gj: int,
    *,
    prefer: set[tuple[int, int]] | None = None,
) -> tuple[int, int] | None:
    height, width = free.shape
    if _in_bounds(gi, gj, width, height) and free[gj, gi]:
        cell = (gi, gj)
        if prefer is None or cell in prefer:
            return cell
    seen = set()
    queue: deque[tuple[int, int, int]] = deque([(gi, gj, 0)])
    fallback: tuple[int, int] | None = None
    while queue:
        ci, cj, depth = queue.popleft()
        if depth > MAX_SNAP_RADIUS:
            continue
        key = (ci, cj)
        if key in seen:
            continue
        seen.add(key)
        if _in_bounds(ci, cj, width, height) and free[cj, ci]:
            if prefer is None or key in prefer:
                return key
            if fallback is None:
                fallback = key
        for di, dj, _ in _NEIGHBORS[:4]:
            queue.append((ci + di, cj + dj, depth + 1))
    return fallback


def _octile(gi: int, gj: int, ti: int, tj: int) -> float:
    dx = abs(gi - ti)
    dy = abs(gj - tj)
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


def _in_bounds(gi: int, gj: int, width: int, height: int) -> bool:
    return 0 <= gi < width and 0 <= gj < height


def _diagonal_blocked(free: np.ndarray, ci: int, cj: int, di: int, dj: int) -> bool:
    if di == 0 or dj == 0:
        return False
    return not (free[cj, ci + di] and free[cj + dj, ci])


def _line_of_sight(free: np.ndarray, a: tuple[int, int], b: tuple[int, int]) -> bool:
    """True if the straight segment a->b stays entirely inside free cells.

    Integer Theta*-style supercover check (Nash et al.): every grid cell the
    segment passes through must be navigable, including the corner-cut guard so
    the line never squeezes between two diagonally-touching blocked cells.
    """
    height, width = free.shape

    def blocked(x: int, y: int) -> bool:
        return not (0 <= x < width and 0 <= y < height) or not free[y, x]

    x0, y0 = a
    x1, y1 = b
    dx = x1 - x0
    dy = y1 - y0
    sx = 1 if dx >= 0 else -1
    sy = 1 if dy >= 0 else -1
    dx = abs(dx)
    dy = abs(dy)
    f = 0
    if dx >= dy:
        while x0 != x1:
            f += dy
            if f >= dx:
                if blocked(x0 + (sx - 1) // 2, y0 + (sy - 1) // 2):
                    return False
                y0 += sy
                f -= dx
            if f != 0 and blocked(x0 + (sx - 1) // 2, y0 + (sy - 1) // 2):
                return False
            if dy == 0 and blocked(x0 + (sx - 1) // 2, y0) and blocked(x0 + (sx - 1) // 2, y0 - 1):
                return False
            x0 += sx
    else:
        while y0 != y1:
            f += dx
            if f >= dy:
                if blocked(x0 + (sx - 1) // 2, y0 + (sy - 1) // 2):
                    return False
                x0 += sx
                f -= dy
            if f != 0 and blocked(x0 + (sx - 1) // 2, y0 + (sy - 1) // 2):
                return False
            if dx == 0 and blocked(x0, y0 + (sy - 1) // 2) and blocked(x0 - 1, y0 + (sy - 1) // 2):
                return False
            y0 += sy
    return True


def _string_pull(cells: list[tuple[int, int]], free: np.ndarray) -> list[tuple[int, int]]:
    """Simple any-angle smoothing: drop interior cells the anchor can see past.

    Collapses the 8-connected A* staircase into straight segments, keeping a
    vertex only where the straight shortcut would leave ``free`` (so the line
    never clips a wall). Centering still comes from the A* cost field choosing
    which cells to route through; this just straightens the result.
    """
    if len(cells) <= 2:
        return cells
    out = [cells[0]]
    anchor = 0
    for i in range(1, len(cells) - 1):
        if not _line_of_sight(free, cells[anchor], cells[i + 1]):
            out.append(cells[i])
            anchor = i
    out.append(cells[-1])
    return out


def _clearance_m(free: np.ndarray, resolution: float) -> np.ndarray:
    """Per-cell distance (m) to the nearest non-navigable cell (inflated wall edge)."""
    if not free.any():
        return np.zeros(free.shape, dtype=np.float32)
    return (cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 3) * resolution).astype(np.float32)


def _clearance_cost(
    clearance_m: np.ndarray, free: np.ndarray, weight: float, target_m: float
) -> np.ndarray:
    """Centering field: linear penalty for being closer than ``target_m`` to a wall.

    The penalty ramps from ``weight`` right at the wall down to ``0`` once a cell
    is at least ``target_m`` away. Because the cost keeps decreasing across the
    whole 0..target range (unlike an exponential that saturates a few cm out),
    A* maximizes wall distance: in corridors narrower than ``2 * target_m`` the
    cheapest route is the centre line; in wider spaces it just holds that much
    clearance. Penalty is always >= 0, so the octile heuristic stays admissible.
    """
    if weight <= 0 or target_m <= 0 or not free.any():
        return np.zeros(free.shape, dtype=np.float32)
    deficit = np.clip(1.0 - clearance_m / target_m, 0.0, 1.0)
    cost = weight * deficit
    cost[~free] = 0.0
    return cost.astype(np.float32)


def _astar(
    free: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    cost_field: np.ndarray | None = None,
) -> list[tuple[int, int]] | None:
    width = free.shape[1]
    height = free.shape[0]
    sg, tg = start, goal
    if not free[sg[1], sg[0]] or not free[tg[1], tg[0]]:
        return None

    open_heap: list[tuple[float, int, tuple[int, int]]] = []
    counter = 0
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    heapq.heappush(open_heap, (_octile(*start, *goal), counter, start))
    closed: set[tuple[int, int]] = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        closed.add(current)
        cg = g_score[current]
        ci, cj = current
        for di, dj, cost in _NEIGHBORS:
            ni, nj = ci + di, cj + dj
            if not _in_bounds(ni, nj, width, height) or not free[nj, ni]:
                continue
            if _diagonal_blocked(free, ci, cj, di, dj):
                continue
            nxt = (ni, nj)
            if nxt in closed:
                continue
            step = cost
            if cost_field is not None:
                # Penalty only adds to cost, so the octile heuristic stays admissible.
                step = cost * (1.0 + float(cost_field[nj, ni]))
            tg_score = cg + step
            if tg_score < g_score.get(nxt, float("inf")):
                g_score[nxt] = tg_score
                came_from[nxt] = current
                counter += 1
                f = tg_score + _octile(ni, nj, goal[0], goal[1])
                heapq.heappush(open_heap, (f, counter, nxt))
    return None


def _simplify_path(points: list[tuple[float, float]], min_step_m: float = 0.25) -> list[tuple[float, float]]:
    if len(points) <= 2:
        return points
    out = [points[0]]
    for pt in points[1:-1]:
        lx, ly = out[-1]
        if math.hypot(pt[0] - lx, pt[1] - ly) >= min_step_m:
            out.append(pt)
    if math.hypot(out[-1][0] - points[-1][0], out[-1][1] - points[-1][1]) > 1e-6:
        out.append(points[-1])
    else:
        out[-1] = points[-1]
    return out


def _smooth_corners(
    pts: list[tuple[float, float]],
    free: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    iters: int,
    ratio: float,
) -> list[tuple[float, float]]:
    """Round hard corners into natural curves via clearance-safe corner cutting.

    Each interior vertex is replaced by two points trimmed ``ratio`` of the way
    along its incoming/outgoing edges (Chaikin-style); repeating ``iters`` times
    approximates a smooth quadratic B-spline. A cut is only accepted when the new
    chord keeps line-of-sight through ``free`` (>= robot footprint from walls),
    otherwise the sharp vertex is kept — so smoothing never clips a wall.
    """
    ratio = min(max(ratio, 0.0), 0.5)
    if iters <= 0 or ratio <= 0.0 or len(pts) <= 2:
        return pts

    def cell(p: tuple[float, float]) -> tuple[int, int]:
        return _world_to_cell(p[0], p[1], origin_x, origin_y, resolution)

    out_pts = list(pts)
    for _ in range(iters):
        if len(out_pts) <= 2:
            break
        rounded: list[tuple[float, float]] = [out_pts[0]]
        for i in range(1, len(out_pts) - 1):
            prev, cur, nxt = out_pts[i - 1], out_pts[i], out_pts[i + 1]
            a = (cur[0] + ratio * (prev[0] - cur[0]), cur[1] + ratio * (prev[1] - cur[1]))
            b = (cur[0] + ratio * (nxt[0] - cur[0]), cur[1] + ratio * (nxt[1] - cur[1]))
            if _line_of_sight(free, cell(a), cell(b)):
                rounded.append(a)
                rounded.append(b)
            else:
                rounded.append(cur)
        rounded.append(out_pts[-1])
        out_pts = rounded
    return out_pts


def plan_route_on_floorplan(
    floorplan: dict[str, Any],
    start_x: float,
    start_y: float,
    destinations: list[tuple[float, float]],
    *,
    extra_obstacles: np.ndarray | None = None,
) -> dict[str, Any]:
    """Plan A* route from start through each destination in order.

    ``extra_obstacles`` is an optional bool grid (H×W, same shape as the floor-plan
    walls) of live obstacles to treat as blocked in addition to the saved walls.
    """
    if not destinations:
        return {"ok": True, "points": [], "point_count": 0, "cell_count": 0}

    legs = [(start_x, start_y)] + list(destinations)
    combined: list[tuple[float, float]] = []
    total_cells = 0

    for i in range(len(legs) - 1):
        sx, sy = legs[i]
        gx, gy = legs[i + 1]
        leg = plan_path_on_floorplan(floorplan, sx, sy, gx, gy, extra_obstacles=extra_obstacles)
        if not leg.get("ok"):
            reason = leg.get("reason") or "no_path"
            return {
                "ok": False,
                "reason": reason,
                "failed_at_destination": i,
                "points": [],
            }
        pts = [(p["x"], p["y"]) for p in leg["points"]]
        total_cells += int(leg.get("cell_count") or 0)
        if combined and pts:
            pts = pts[1:]
        combined.extend(pts)

    combined = _simplify_path(combined, min_step_m=0.25)

    # Round the corners of the stitched route into a smooth, natural curve. Done
    # last (after stitching/simplifying) so it isn't decimated downstream; the
    # follower drives this geometry directly.
    walls, zones, origin_x, origin_y, resolution, _, _ = decode_grids(floorplan)
    if extra_obstacles is not None and extra_obstacles.shape == walls.shape:
        walls = walls.copy()
        walls[extra_obstacles] = 1
    clearance = configured_planner_clearance()
    free = _navigable_grid(walls, zones, resolution, robot_radius_m=clearance["robot_radius_m"])
    combined = _smooth_corners(
        combined,
        free,
        origin_x,
        origin_y,
        resolution,
        clearance["corner_smoothing_iters"],
        clearance["corner_smoothing_ratio"],
    )

    points = [{"x": round(x, 4), "y": round(y, 4)} for x, y in combined]
    return {
        "ok": True,
        "points": points,
        "cell_count": total_cells,
        "point_count": len(points),
    }


def plan_path_on_floorplan(
    floorplan: dict[str, Any],
    start_x: float,
    start_y: float,
    goal_x: float,
    goal_y: float,
    *,
    extra_obstacles: np.ndarray | None = None,
) -> dict[str, Any]:
    walls, zones, origin_x, origin_y, resolution, width, height = decode_grids(floorplan)
    if extra_obstacles is not None and extra_obstacles.shape == walls.shape:
        walls = walls.copy()
        walls[extra_obstacles] = 1

    sgi, sgj = _world_to_cell(start_x, start_y, origin_x, origin_y, resolution)
    ggi, ggj = _world_to_cell(goal_x, goal_y, origin_x, origin_y, resolution)

    clearance = configured_planner_clearance()

    # Single hard inflation pass: walls grow by the robot footprint and nothing
    # tighter is ever entered. The soft clearance cost then centres the path
    # wherever there's room, and string-pulling straightens the result.
    free = _navigable_grid(walls, zones, resolution, robot_radius_m=clearance["robot_radius_m"])
    start = _nearest_free(free, sgi, sgj)
    if start is None:
        return {"ok": False, "reason": "start_not_navigable", "points": []}
    start_region = _component_cells(free, start)
    goal = _nearest_free(free, ggi, ggj, prefer=start_region)
    if goal is None:
        return {"ok": False, "reason": "goal_not_navigable", "points": []}

    clearance_m = _clearance_m(free, resolution)
    cost_field = _clearance_cost(
        clearance_m, free, clearance["centering_weight"], clearance["centering_clearance_m"]
    )
    cells = _astar(free, start, goal, cost_field=cost_field)
    if cells is None:
        reason = "disconnected" if goal not in start_region else "no_path"
        return {"ok": False, "reason": reason, "points": []}

    grid_cell_count = len(cells)
    cells = _string_pull(cells, free)
    world_pts = [_cell_to_world(gi, gj, origin_x, origin_y, resolution) for gi, gj in cells]
    world_pts = _simplify_path(world_pts, min_step_m=max(0.2, resolution * 3))

    points = [{"x": round(x, 4), "y": round(y, 4)} for x, y in world_pts]
    return {
        "ok": True,
        "points": points,
        "cell_count": grid_cell_count,
        "point_count": len(points),
    }
