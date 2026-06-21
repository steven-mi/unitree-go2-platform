"""Manual teleop via MCF Move commands."""

from __future__ import annotations

from config import cfg_float, load_config

_CFG = load_config()

MAX_VX = cfg_float(_CFG, "teleop.max_vx")
MAX_VY = cfg_float(_CFG, "teleop.max_vy")
MAX_VYAW = cfg_float(_CFG, "teleop.max_vyaw")


def clamp_drive(vx: float, vy: float, vyaw: float) -> tuple[float, float, float]:
    return (
        max(-MAX_VX, min(MAX_VX, vx)),
        max(-MAX_VY, min(MAX_VY, vy)),
        max(-MAX_VYAW, min(MAX_VYAW, vyaw)),
    )


def send_drive(pub, vx: float, vy: float, vyaw: float) -> None:
    from live.navigation import sport_move_no_reply

    vx, vy, vyaw = clamp_drive(vx, vy, vyaw)
    sport_move_no_reply(pub, vx, vy, vyaw)
