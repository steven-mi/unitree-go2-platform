"""Shared FastAPI dependencies."""

from __future__ import annotations

from functools import lru_cache

from config import FLOOR_PLAN_REV
from replay.session import SessionReplay
from replay.store import load_session


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
