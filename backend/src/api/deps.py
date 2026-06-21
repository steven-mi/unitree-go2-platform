"""Shared FastAPI dependencies and request guards."""

from __future__ import annotations

from functools import lru_cache
from typing import Annotated, Any

from fastapi import Depends, HTTPException

from config import FLOOR_PLAN_REV
from live.manager import LiveStatus, live_manager
from replay.session import SessionReplay
from replay.store import load_session
from scan import store as scan_store


@lru_cache(maxsize=16)
def get_session(session_id: str) -> SessionReplay:
    return load_session(session_id)


def invalidate_session_cache() -> None:
    get_session.cache_clear()


def reset_stale_floor_plan(session: SessionReplay) -> None:
    if getattr(session, "_floor_plan_rev", 0) != FLOOR_PLAN_REV:
        session._floor_builder = None
        session._floor_plan_rev = 0
        session._floor_synced_t = -1.0
        session._floor_lidar_i = 0
        session._floor_odom_i = 0


def require_connected() -> LiveStatus:
    status = live_manager.status()
    if not status.connected:
        raise HTTPException(503, "Not connected to robot")
    return status


def resolve_scan(scan_id: str) -> dict[str, Any]:
    """Return scan metadata, self-healing the `latest` alias, or raise 404/400."""
    try:
        if scan_id == scan_store.LATEST_SCAN_ID:
            return scan_store.ensure_latest_scan()
        return scan_store.load_scan_meta(scan_id)
    except FileNotFoundError:
        raise HTTPException(404, "Scan not found")
    except ValueError as exc:
        raise HTTPException(400, str(exc))


def resolve_session(session_id: str) -> SessionReplay:
    try:
        return get_session(session_id)
    except FileNotFoundError:
        raise HTTPException(404, "Session not found")
    except ValueError as exc:
        raise HTTPException(400, str(exc))


ConnectedStatus = Annotated[LiveStatus, Depends(require_connected)]
ScanMeta = Annotated[dict, Depends(resolve_scan)]
SessionDep = Annotated[SessionReplay, Depends(resolve_session)]
