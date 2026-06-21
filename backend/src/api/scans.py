"""Saved scan floor plans and waypoint routes."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from api import scan_ops
from api.deps import ConnectedStatus, ScanMeta
from api.schemas import PathPointBody
from config import scans_root
from live.manager import live_manager
from scan import store as scan_store

router = APIRouter(prefix="/api/scans", tags=["scans"])


class RestoreScanBody(BaseModel):
    scan_id: str


class SavePathBody(BaseModel):
    route: list[PathPointBody] | None = None
    destinations: list[PathPointBody] | None = None


class PlanRouteBody(BaseModel):
    start_x: float
    start_y: float
    destinations: list[PathPointBody]
    save: bool = True


def _current_odom_pose() -> dict[str, float] | None:
    try:
        pose = live_manager.frame_at().get("pose")
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
def api_sync_latest_scan(status: ConnectedStatus):
    floorplan = live_manager.build_floor_plan()
    if int(floorplan.get("scan_count") or 0) == 0:
        return scan_store.ensure_latest_scan()

    source_session_id = status.session_id if status.recording else None
    odom_origin = _current_odom_pose()
    scan_store.ensure_latest_scan()
    live_manager.save_floor_grid(scans_root() / scan_store.LATEST_SCAN_ID)
    return scan_store.update_latest_scan(
        floorplan,
        source_session_id=source_session_id,
        odom_origin=odom_origin,
    )


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
def api_reset_latest_scan(_status: ConnectedStatus):
    meta, archived_id = scan_store.reset_latest_scan()
    live_manager.reset_scan_epoch()
    return {"latest": meta, "archived_id": archived_id}


@router.get("/{scan_id}")
def api_get_scan(meta: ScanMeta):
    return meta


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
def api_localize_scan(scan_id: str, _status: ConnectedStatus, apply: bool = True):
    return scan_ops.localize_scan(scan_id, apply)


@router.get("/{scan_id}/floorplan")
def api_scan_floorplan(scan_id: str, _meta: ScanMeta):
    try:
        return scan_store.load_floorplan(scan_id)
    except FileNotFoundError:
        raise HTTPException(404, "Scan or floor plan not found")
    except ValueError as exc:
        raise HTTPException(400, str(exc))


@router.post("/{scan_id}/plan-route")
def api_plan_route(scan_id: str, _meta: ScanMeta, body: PlanRouteBody):
    if not body.destinations:
        raise HTTPException(400, "At least one destination is required")
    destinations = [{"x": round(p.x, 4), "y": round(p.y, 4)} for p in body.destinations]
    return scan_ops.plan_route_for_scan(
        scan_id, body.start_x, body.start_y, destinations, body.save
    )


@router.get("/{scan_id}/path")
def api_get_path(scan_id: str, _meta: ScanMeta):
    data = scan_store.load_path_data(scan_id)
    return {"route": data["route"], "destinations": data["destinations"]}


@router.put("/{scan_id}/path")
def api_save_path(scan_id: str, _meta: ScanMeta, body: SavePathBody):
    saved = scan_store.save_path(
        scan_id,
        [{"x": p.x, "y": p.y} for p in (body.route or [])],
        destinations=(
            [{"x": p.x, "y": p.y} for p in body.destinations]
            if body.destinations
            else None
        ),
    )
    return {"route": saved["route"], "destinations": saved["destinations"]}


@router.delete("/{scan_id}/path")
def api_clear_path(scan_id: str, _meta: ScanMeta):
    scan_store.save_path(scan_id, [], destinations=[])
    return {"ok": True}
