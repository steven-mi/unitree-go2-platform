"""Live robot connection routes."""

from __future__ import annotations

import json
from dataclasses import asdict

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import Response
from pydantic import BaseModel

from api.deps import ConnectedStatus, invalidate_session_cache
from api.schemas import PathPointBody
from live.manager import live_manager


class FollowPathBody(BaseModel):
    points: list[PathPointBody]
    map_frame: bool = False


class SportCommandBody(BaseModel):
    command: str
    parameter: dict | None = None


router = APIRouter(prefix="/api/live", tags=["live"])


def _live_status_dict() -> dict:
    return asdict(live_manager.status())


@router.get("/status")
def api_live_status():
    return _live_status_dict()


@router.post("/connect")
def api_live_connect(ip: str | None = None):
    try:
        live_manager.connect(ip=ip)
    except Exception as exc:
        raise HTTPException(503, str(exc))
    return _live_status_dict()


@router.post("/disconnect")
def api_live_disconnect():
    live_manager.disconnect()
    return _live_status_dict()


@router.post("/recording/start")
def api_live_start_recording(name: str = "", note: str = ""):
    try:
        live_manager.start_recording(name=name, note=note)
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
    return _live_status_dict()


@router.post("/recording/stop")
def api_live_stop_recording():
    try:
        result = live_manager.stop_recording()
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
    invalidate_session_cache()
    return {"ok": True, **result}


@router.post("/follow-path")
def api_live_follow_path(body: FollowPathBody):
    if not body.points:
        raise HTTPException(400, "Path must have at least one point")
    try:
        return live_manager.follow_path(
            [{"x": p.x, "y": p.y} for p in body.points],
            map_frame=body.map_frame,
        )
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))


@router.get("/navigation")
def api_live_navigation():
    return live_manager.navigation_status()


@router.post("/stop-navigation")
def api_live_stop_navigation():
    live_manager.stop_navigation()
    return {"ok": True, "navigation": live_manager.navigation_status()}


@router.post("/drive/stop")
def api_live_drive_stop():
    try:
        live_manager.stop_drive()
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
    return {"ok": True}


@router.websocket("/drive/ws")
async def api_live_drive_ws(ws: WebSocket):
    """Low-latency teleop stream — client sends {vx, vy, vyaw} at ~50–100 Hz."""
    await ws.accept()
    live_manager.touch_client()
    try:
        while True:
            raw = await ws.receive_text()
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                await ws.send_json({"error": "invalid json"})
                continue
            try:
                live_manager.drive(
                    float(data.get("vx", 0.0)),
                    float(data.get("vy", 0.0)),
                    float(data.get("vyaw", 0.0)),
                )
            except (TypeError, ValueError):
                await ws.send_json({"error": "invalid velocity"})
            except RuntimeError as exc:
                await ws.send_json({"error": str(exc)})
    except WebSocketDisconnect:
        pass
    finally:
        live_manager.stop_drive()


@router.post("/sport")
def api_live_sport(body: SportCommandBody):
    try:
        result = live_manager.sport_command(body.command, body.parameter)
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
    if not result.get("ok"):
        raise HTTPException(502, f"Sport command failed (code {result.get('code', -1)})")
    return {"ok": True, **result}


@router.get("/frame")
def api_live_frame(_status: ConnectedStatus, t: float | None = None):
    return live_manager.frame_at(t)


@router.get("/session")
def api_live_session(_status: ConnectedStatus):
    return live_manager.session_detail()


@router.get("/lidar/{seq}")
def api_live_lidar(seq: int, _status: ConnectedStatus, max_points: int = 0):
    try:
        data = live_manager.load_lidar_points_binary(seq, max_points=max_points)
        return Response(content=data, media_type="application/octet-stream")
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))


@router.get("/floorplan")
def api_live_floorplan(
    _status: ConnectedStatus,
    t: float | None = None,
    x: float | None = None,
    y: float | None = None,
    lidar_seq: int | None = None,
):
    try:
        return live_manager.build_floor_plan(upto_t=t, robot_x=x, robot_y=y, lidar_seq=lidar_seq)
    except ValueError as exc:
        raise HTTPException(400, str(exc))


@router.get("/video/latest.jpg")
def api_live_video_latest(_status: ConnectedStatus):
    data = live_manager.latest_video_jpg()
    if not data:
        raise HTTPException(404, "No video frame yet")
    return Response(
        content=data,
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store"},
    )
