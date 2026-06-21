"""Orchestration for scan route planning and localization.

These functions glue the saved-scan store, the live robot, and the pure
planning/localization solvers together. They live outside the router so the
endpoints in ``api/scans.py`` stay thin.
"""

from __future__ import annotations

import math
from typing import Any

from fastapi import HTTPException

from live.manager import live_manager
from localization.matcher import estimate_map_pose, pose_in_explored
from route_planning.planner import lidar_obstacle_mask, plan_route_on_floorplan
from scan import store as scan_store

_PLAN_MESSAGES = {
    "start_not_navigable": "Start point is inside a wall or outside the map",
    "goal_not_navigable": "Destination is inside a wall or outside the map",
    "no_path": "No route found — try a different destination",
    "disconnected": "No route found — destination is unreachable through narrow passages on this map",
}


def _live_obstacle_mask(scan_id: str, floorplan: dict, start_x: float, start_y: float):
    """Project the connected robot's live lidar into the map frame as fresh obstacles.

    Returns ``None`` when offline or no obstacle stands in explored space, so route
    planning falls back to the saved walls only.
    """
    try:
        if not live_manager.status().connected:
            return None
        points = live_manager.latest_lidar_points()
        if points is None or len(points) == 0:
            return None
        align = scan_store.load_scan_meta(scan_id).get("map_alignment") or {}
        pose = live_manager.current_pose()
        robot_pose = None
        if pose is not None and pose.get("yaw") is not None:
            robot_pose = (
                float(pose.get("x") or 0.0),
                float(pose.get("y") or 0.0),
                float(pose.get("yaw") or 0.0),
            )
        return lidar_obstacle_mask(
            floorplan,
            points,
            tx=float(align.get("tx") or 0.0),
            ty=float(align.get("ty") or 0.0),
            dyaw=float(align.get("dyaw") or 0.0),
            robot_xy=(start_x, start_y),
            robot_pose=robot_pose,
        )
    except Exception:
        return None


def plan_route_for_scan(
    scan_id: str,
    start_x: float,
    start_y: float,
    destinations: list[dict],
    save: bool,
) -> dict[str, Any]:
    try:
        floorplan = scan_store.load_floorplan(scan_id)
    except FileNotFoundError:
        raise HTTPException(
            404,
            "This scan has no floor plan — open Live, build a map, then save the scan again",
        )
    if "walls_b64" not in floorplan:
        raise HTTPException(400, "Scan has no floor plan for path planning")

    dest_tuples = [(float(d["x"]), float(d["y"])) for d in destinations]
    extra_obstacles = _live_obstacle_mask(scan_id, floorplan, start_x, start_y)
    result = plan_route_on_floorplan(
        floorplan, start_x, start_y, dest_tuples, extra_obstacles=extra_obstacles
    )

    if not result.get("ok"):
        reason = result.get("reason") or "planning_failed"
        failed = result.get("failed_at_destination")
        msg = _PLAN_MESSAGES.get(reason, "Path planning failed")
        if failed is not None:
            msg = f"{msg} (leg {failed + 1})"
        raise HTTPException(400, msg)

    route = result["points"]
    if save:
        scan_store.save_path(scan_id, route, destinations=destinations)

    return {
        "route": route,
        "destinations": destinations,
        "point_count": len(route),
        "cell_count": result.get("cell_count", 0),
    }


def localize_scan(scan_id: str, apply: bool) -> dict[str, Any]:
    try:
        floorplan = scan_store.load_floorplan(scan_id)
        meta = scan_store.load_scan_meta(scan_id)
    except FileNotFoundError:
        raise HTTPException(404, "Scan not found")
    except ValueError as exc:
        raise HTTPException(400, str(exc))

    if "walls_b64" not in floorplan:
        raise HTTPException(400, "Scan has no floor plan for localization")

    points = live_manager.latest_lidar_points()
    if points is None or len(points) == 0:
        raise HTTPException(503, "No lidar data yet — wait for a scan")

    pose = live_manager.current_pose()
    if not pose:
        raise HTTPException(503, "No robot pose yet")

    align = meta.get("map_alignment") or {}
    atx = float(align.get("tx") or 0.0)
    aty = float(align.get("ty") or 0.0)
    adyaw = float(align.get("dyaw") or 0.0)
    rx = float(pose["x"])
    ry = float(pose["y"])

    # Prior robot map position from the stored transform: p_map = R(dyaw)·p_odom + t.
    c, s = math.cos(adyaw), math.sin(adyaw)
    prior_x = c * rx - s * ry + atx
    prior_y = s * rx + c * ry + aty

    has_prior = abs(atx) > 1e-6 or abs(aty) > 1e-6 or abs(adyaw) > 1e-6
    if has_prior and not pose_in_explored(prior_x, prior_y, floorplan):
        has_prior = False

    result = estimate_map_pose(
        points,
        floorplan,
        pose,
        hint_x=prior_x if has_prior else None,
        hint_y=prior_y if has_prior else None,
        hint_dyaw=adyaw if has_prior else None,
        search_radius_m=2.5 if has_prior else 5.0,
        yaw_search_rad=1.2 if has_prior else math.pi,
    )

    if apply and result.get("ok"):
        alignment = result["map_alignment"]
        saved = scan_store.update_map_alignment(
            scan_id,
            float(alignment["tx"]),
            float(alignment["ty"]),
            dyaw=float(alignment.get("dyaw") or 0.0),
        )
        result["map_alignment"] = saved

    return result
