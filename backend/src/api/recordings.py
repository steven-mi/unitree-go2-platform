"""Recorded session replay routes."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel

from api.deps import get_session, invalidate_session_cache, reset_stale_floor_plan
from live.manager import live_manager
from replay.store import delete_session, list_sessions, update_session_tags

router = APIRouter(prefix="/api/recordings", tags=["recordings"])


class RecordingTagsBody(BaseModel):
    tags: list[str] = []


@router.get("")
def api_list_recordings():
    return {"sessions": list_sessions()}


@router.get("/{session_id}")
def api_get_recording(session_id: str):
    try:
        session = get_session(session_id)
    except FileNotFoundError:
        raise HTTPException(404, "Session not found")
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    tags = session.manifest.get("tags")
    if tags is None:
        tags = session.session_meta.get("tags", [])
    return {
        "id": session.session_id,
        "manifest": session.manifest,
        "session": session.session_meta,
        "tags": tags if isinstance(tags, list) else [],
        "duration": session.duration,
        "rpc": session.rpc,
        "services": session.services,
        "streams": {
            "video": len(session.video),
            "lidar": len(session.lidar),
            "odom": len(session.odom),
            "uwb": len(session.uwb),
            "audio": len(session.audio_hub),
        },
    }


@router.put("/{session_id}/tags")
def api_update_recording_tags(session_id: str, body: RecordingTagsBody):
    try:
        tags = update_session_tags(session_id, body.tags)
    except FileNotFoundError:
        raise HTTPException(404, "Session not found")
    invalidate_session_cache()
    return {"id": session_id, "tags": tags}


@router.delete("/{session_id}")
def api_delete_recording(session_id: str):
    if live_manager.status().session_id == session_id:
        raise HTTPException(409, "Cannot delete a recording while it is being recorded")
    try:
        delete_session(session_id)
    except FileNotFoundError:
        raise HTTPException(404, "Session not found")
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    invalidate_session_cache()
    return {"ok": True}


@router.get("/{session_id}/frame")
def api_frame_at(session_id: str, t: float = 0.0):
    try:
        session = get_session(session_id)
    except FileNotFoundError:
        raise HTTPException(404, "Session not found")
    return session.frame_at(t)


@router.get("/{session_id}/lidar/{seq}")
def api_lidar_points(session_id: str, seq: int, max_points: int = 0):
    try:
        session = get_session(session_id)
    except FileNotFoundError:
        raise HTTPException(404, "Session not found")
    try:
        data = session.load_lidar_points_binary(seq, max_points=max_points)
        return Response(content=data, media_type="application/octet-stream")
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))


@router.get("/{session_id}/floorplan")
def api_floor_plan(
    session_id: str,
    t: float | None = None,
    x: float | None = None,
    y: float | None = None,
    lidar_seq: int | None = None,
):
    try:
        session = get_session(session_id)
    except FileNotFoundError:
        raise HTTPException(404, "Session not found")
    reset_stale_floor_plan(session)
    try:
        return session.build_floor_plan(upto_t=t, robot_x=x, robot_y=y, lidar_seq=lidar_seq)
    except ValueError as exc:
        raise HTTPException(400, str(exc))


@router.get("/{session_id}/video/{filename}")
def api_video_file(session_id: str, filename: str):
    try:
        session = get_session(session_id)
    except FileNotFoundError:
        raise HTTPException(404, "Session not found")
    if ".." in filename or "/" in filename:
        raise HTTPException(400, "Invalid filename")
    path = session.root / "video" / filename
    if not path.exists():
        raise HTTPException(404, "Frame not found")
    return FileResponse(path, media_type="image/jpeg")
