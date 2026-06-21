"""Saved scan floor plans and waypoint routes."""

from __future__ import annotations

import logging
import math
from typing import Any

import numpy as np
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from config import scans_root
from live.manager import live_manager
from localization.matcher import estimate_map_pose, pose_in_explored
from route_planning.planner import lidar_obstacle_mask, plan_route_on_floorplan
from scan import store as scan_store

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/scans", tags=["scans"])


class RestoreScanBody(BaseModel):
    scan_id: str


class PathPointBody(BaseModel):
    x: float
    y: float


class SavePathBody(BaseModel):
    route: list[PathPointBody] | None = None
    destinations: list[PathPointBody] | None = None
    points: list[PathPointBody] | None = None


class PlanRouteBody(BaseModel):
    start_x: float
    start_y: float
    destinations: list[PathPointBody]
    save: bool = True


def _plan_messages() -> dict[str, str]:
    return {
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
        align = (scan_store.load_scan_meta(scan_id).get("map_alignment") or {})
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


def _run_plan_route(scan_id: str, start_x: float, start_y: float, destinations: list[dict], save: bool):
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
    obs_count = int(extra_obstacles.sum()) if extra_obstacles is not None else 0
    result = plan_route_on_floorplan(
        floorplan, start_x, start_y, dest_tuples, extra_obstacles=extra_obstacles
    )

    # --- TEMP diagnostics for "can't find a path" investigation ---
    pts = result.get("points") or []
    snap_m = -1.0
    if pts:
        ex, ey = pts[-1]["x"], pts[-1]["y"]
        gx, gy = dest_tuples[-1]
        snap_m = math.hypot(ex - gx, ey - gy)
    logger.warning(
        "[plan-route] start=(%.2f,%.2f) dests=%s ok=%s reason=%s pts=%d live_obstacle_cells=%d goal_snap=%.2fm",
        start_x, start_y,
        [(round(x, 2), round(y, 2)) for x, y in dest_tuples],
        result.get("ok"), result.get("reason"), len(pts), obs_count, snap_m,
    )
    if not result.get("ok") and obs_count > 0:
        # Shadow-plan ignoring live obstacles to isolate the cause.
        shadow = plan_route_on_floorplan(floorplan, start_x, start_y, dest_tuples, extra_obstacles=None)
        bbox = None
        if extra_obstacles is not None and extra_obstacles.any():
            ys, xs = np.where(extra_obstacles)
            bbox = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
        logger.warning(
            "[plan-route] WITHOUT live obstacles: ok=%s reason=%s pts=%d | obstacle_bbox_cells=%s",
            shadow.get("ok"), shadow.get("reason"), len(shadow.get("points") or []), bbox,
        )

    if not result.get("ok"):
        reason = result.get("reason") or "planning_failed"
        failed = result.get("failed_at_destination")
        msg = _plan_messages().get(reason, "Path planning failed")
        if failed is not None:
            msg = f"{msg} (leg {failed + 1})"
        raise HTTPException(400, msg)

    route = result["points"]
    if save:
        scan_store.save_path(scan_id, route, destinations=destinations)

    return {
        "route": route,
        "destinations": destinations,
        "points": route,
        "point_count": len(route),
        "cell_count": result.get("cell_count", 0),
    }


def _current_odom_pose() -> dict[str, float] | None:
    try:
        frame = live_manager.frame_at()
        pose = frame.get("pose")
        if not pose:
            return None
        return {
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "yaw": float(pose.get("yaw") or 0.0),
        }
    except Exception:
        return None


@router.get("")
def api_list_scans():
    return {"scans": scan_store.list_scans()}


@router.put("/latest/sync")
def api_sync_latest_scan():
    status = live_manager.status()
    if not status.connected:
        raise HTTPException(503, "Not connected to robot")

    floorplan = live_manager.build_floor_plan()
    mem_count = int(floorplan.get("scan_count") or 0)
    if mem_count == 0:
        return scan_store.ensure_latest_scan()

    source_session_id = status.session_id if status.recording else None
    odom_origin = _current_odom_pose()
    scan_store.ensure_latest_scan()
    scan_path = scans_root() / scan_store.LATEST_SCAN_ID
    live_manager.save_floor_grid(scan_path)

    meta = scan_store.update_latest_scan(
        floorplan,
        source_session_id=source_session_id,
        odom_origin=odom_origin,
    )
    return meta


@router.post("/latest/restore")
def api_restore_latest_scan(body: RestoreScanBody):
    if body.scan_id == scan_store.LATEST_SCAN_ID:
        raise HTTPException(400, "Scan is already the active map")

    try:
        meta, archived_id = scan_store.restore_scan_to_latest(body.scan_id)
    except FileNotFoundError:
        raise HTTPException(404, "Scan not found")
    except ValueError as exc:
        raise HTTPException(400, str(exc))

    if live_manager.status().connected:
        live_manager.load_latest_scan_session()

    return {"latest": meta, "archived_id": archived_id}


@router.post("/latest/reset")
def api_reset_latest_scan():
    status = live_manager.status()
    if not status.connected:
        raise HTTPException(503, "Not connected to robot")

    meta, archived_id = scan_store.reset_latest_scan()
    live_manager.reset_scan_epoch()
    return {"latest": meta, "archived_id": archived_id}


@router.get("/{scan_id}")
def api_get_scan(scan_id: str):
    try:
        if scan_id == scan_store.LATEST_SCAN_ID:
            return scan_store.ensure_latest_scan()
        return scan_store.load_scan_meta(scan_id)
    except FileNotFoundError:
        raise HTTPException(404, "Scan not found")
    except ValueError as exc:
        raise HTTPException(400, str(exc))


@router.delete("/{scan_id}")
def api_delete_scan(scan_id: str):
    if scan_id == scan_store.LATEST_SCAN_ID:
        raise HTTPException(400, "Cannot delete the active latest scan — use reset instead")
    try:
        scan_store.delete_scan(scan_id)
    except FileNotFoundError:
        raise HTTPException(404, "Scan not found")
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    return {"ok": True}


@router.post("/{scan_id}/localize")
def api_localize_scan(scan_id: str, apply: bool = True):
    status = live_manager.status()
    if not status.connected:
        raise HTTPException(503, "Not connected to robot")

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


@router.get("/{scan_id}/floorplan")
def api_scan_floorplan(scan_id: str):
    try:
        # `latest` is briefly archived + recreated during a scan reset; self-heal
        # so polling clients see an empty plan (200) instead of a transient 404.
        if scan_id == scan_store.LATEST_SCAN_ID:
            scan_store.ensure_latest_scan()
        return scan_store.load_floorplan(scan_id)
    except FileNotFoundError:
        raise HTTPException(404, "Scan or floor plan not found")
    except ValueError as exc:
        raise HTTPException(400, str(exc))


@router.post("/{scan_id}/plan-route")
def api_plan_route(scan_id: str, body: PlanRouteBody):
    try:
        scan_store.load_scan_meta(scan_id)
    except FileNotFoundError:
        raise HTTPException(404, "Scan not found")
    except ValueError as exc:
        raise HTTPException(400, str(exc))

    if not body.destinations:
        raise HTTPException(400, "At least one destination is required")

    destinations = [{"x": round(p.x, 4), "y": round(p.y, 4)} for p in body.destinations]
    return _run_plan_route(scan_id, body.start_x, body.start_y, destinations, body.save)


@router.get("/{scan_id}/path")
def api_get_path(scan_id: str):
    try:
        # See api_scan_floorplan: self-heal `latest` across a scan reset.
        if scan_id == scan_store.LATEST_SCAN_ID:
            scan_store.ensure_latest_scan()
        scan_store.load_scan_meta(scan_id)
    except FileNotFoundError:
        raise HTTPException(404, "Scan not found")
    data = scan_store.load_path_data(scan_id)
    return {
        "route": data["route"],
        "destinations": data["destinations"],
        "points": data["route"],
    }


@router.put("/{scan_id}/path")
def api_save_path(scan_id: str, body: SavePathBody):
    try:
        scan_store.load_scan_meta(scan_id)
        route_pts = body.route or body.points or []
        dest_pts = body.destinations
        saved = scan_store.save_path(
            scan_id,
            [{"x": p.x, "y": p.y} for p in route_pts],
            destinations=[{"x": p.x, "y": p.y} for p in dest_pts] if dest_pts else None,
        )
    except FileNotFoundError:
        raise HTTPException(404, "Scan not found")
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    return {
        "route": saved["route"],
        "destinations": saved["destinations"],
        "points": saved["route"],
    }


@router.delete("/{scan_id}/path")
def api_clear_path(scan_id: str):
    try:
        scan_store.load_scan_meta(scan_id)
        scan_store.save_path(scan_id, [], destinations=[])
    except FileNotFoundError:
        raise HTTPException(404, "Scan not found")
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    return {"ok": True}
