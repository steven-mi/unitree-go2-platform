"""Application settings persisted in config.yml."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from config import configured_aes_128_key, configured_robot_ip, set_app_settings
from live.manager import live_manager

router = APIRouter(prefix="/api/settings", tags=["settings"])


class SettingsResponse(BaseModel):
    robot_ip: str
    aes_128_key: str | None = None


class SettingsUpdateBody(BaseModel):
    robot_ip: str = Field(min_length=1)
    aes_128_key: str | None = None


@router.get("")
def get_settings() -> SettingsResponse:
    return SettingsResponse(
        robot_ip=configured_robot_ip(),
        aes_128_key=configured_aes_128_key(),
    )


@router.put("")
def update_settings(body: SettingsUpdateBody) -> SettingsResponse:
    try:
        ip, key = set_app_settings(robot_ip=body.robot_ip, aes_128_key=body.aes_128_key)
    except ValueError as exc:
        raise HTTPException(400, str(exc)) from exc

    live_manager.set_robot_ip(ip)
    return SettingsResponse(robot_ip=ip, aes_128_key=key)
