"""Segment navigation: plan turn + forward legs on our side, timed Move commands to the dog."""

from __future__ import annotations

import asyncio
import json
import logging
import math
import random
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np

_obstacle_log = logging.getLogger("nav.obstacle")
_last_obstacle_log_t = 0.0

from unitree_webrtc_connect.constants import (
    OBSTACLES_AVOID_API,
    RTC_TOPIC,
    SPORT_CMD_MCF,
)

from config import cfg_float, cfg_int, load_config

_CFG = load_config()

MOVE_HZ = cfg_float(_CFG, "nav.move_hz")
FORWARD_SPEED = cfg_float(_CFG, "nav.speed")  # fallback default; overridden per-run by configured_nav_speed()
TURN_SPEED = cfg_float(_CFG, "nav.turn_speed")  # rad/s cap (matches teleop max_vyaw)

# Navigation drives through the obstacle-avoidance joystick channel
# (rt/wirelesscontroller) so the Go2 firmware stops/filters for obstacles the
# onboard L1 lidar can't see up close. Joystick axes are normalized [-1, 1];
# these scales map our velocity intent to full-stick deflection and were
# calibrated live (pure-rotation test: rx 0.47 -> ~1.17 rad/s).
WIRELESS_VX_FULL = cfg_float(_CFG, "nav.wireless_vx_full")
WIRELESS_VY_FULL = cfg_float(_CFG, "nav.wireless_vy_full")
WIRELESS_VYAW_FULL = cfg_float(_CFG, "nav.wireless_vyaw_full")

# Closed-loop path tracking
WAYPOINT_REACHED_M = cfg_float(_CFG, "nav.waypoint_reached_m")
LOOKAHEAD_M = cfg_float(_CFG, "nav.lookahead_m")
SLOW_DIST_M = cfg_float(_CFG, "nav.slow_dist_m")
KP_YAW = cfg_float(_CFG, "nav.kp_yaw")
POSE_MAX_AGE_S = cfg_float(_CFG, "nav.pose_max_age_s")
STUCK_TIMEOUT_S = cfg_float(_CFG, "nav.stuck_timeout_s")
STUCK_RECOVER_S = cfg_float(_CFG, "nav.stuck_recover_s")
TURN_IN_PLACE_RAD = cfg_float(_CFG, "nav.turn_in_place_rad")
TURN_RESUME_RAD = cfg_float(_CFG, "nav.turn_resume_rad")

# Obstacle pause — secondary, lidar-only. Primary stopping is handled by the
# Go2 firmware avoidance via the wireless channel. (range_obstacle is not
# reported on this unit, so we no longer read it.)
OBSTACLE_STOP_M = cfg_float(_CFG, "obstacle.stop_m")
OBSTACLE_CLEAR_M = cfg_float(_CFG, "obstacle.clear_m")
OBSTACLE_CLEAR_S = cfg_float(_CFG, "obstacle.clear_s")
OBSTACLE_WAIT_POLL_S = cfg_float(_CFG, "obstacle.wait_poll_s")
OBSTACLE_PAUSE_MAX_S = cfg_float(_CFG, "obstacle.pause_max_s")
LIDAR_BODY_MIN_X = cfg_float(_CFG, "obstacle.body_min_x")
LIDAR_CONE_HALF_M = cfg_float(_CFG, "obstacle.cone_half_m")
OBSTACLE_Z_MIN = cfg_float(_CFG, "obstacle.z_min")
OBSTACLE_Z_MAX = cfg_float(_CFG, "obstacle.z_max")
LIDAR_OBSTACLE_MIN_POINTS = cfg_int(_CFG, "obstacle.min_points")
FORWARD_BLOCK_MIN_VX = cfg_float(_CFG, "obstacle.forward_block_min_vx")
FINAL_APPROACH_MARGIN_M = cfg_float(_CFG, "obstacle.final_approach_margin_m")

PoseGetter = Callable[[], dict[str, Any] | None]
ShouldContinue = Callable[[], bool]
LidarGetter = Callable[[], np.ndarray | None]
OnNavPhase = Callable[[str], None]  # "running" | "paused_obstacle" | "paused_localization"
# Maps an (N,3) odom-frame lidar array to a bool mask marking points that fall on
# known map walls. The safety cone drops those so it only stops for unmapped
# obstacles the planner couldn't already route around (see make_known_wall_filter).
WallFilter = Callable[[np.ndarray], np.ndarray]


@dataclass(frozen=True)
class NavSegment:
    kind: Literal["turn", "forward"]
    value: float  # radians or meters


def angle_diff(from_yaw: float, to_yaw: float) -> float:
    return (to_yaw - from_yaw + math.pi) % (2 * math.pi) - math.pi


def dist_xy(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(bx - ax, by - ay)


def _body_xy_from_lidar(
    points: np.ndarray,
    pose: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """ULIDAR points are in odom frame — subtract pose, then rotate into body frame."""
    px = float(pose.get("x") or 0.0)
    py = float(pose.get("y") or 0.0)
    pz = float(pose.get("z") or 0.0)
    yaw = float(pose.get("yaw") or 0.0)
    c = math.cos(yaw)
    s = math.sin(yaw)
    wx = points[:, 0].astype(np.float64) - px
    wy = points[:, 1].astype(np.float64) - py
    bx = c * wx + s * wy
    by = -s * wx + c * wy
    bz = points[:, 2].astype(np.float64) - pz
    return bx, by, bz


def _lidar_cone_blocked(
    bx: np.ndarray,
    by: np.ndarray,
    bz: np.ndarray,
    *,
    stop_m: float,
) -> bool:
    height_ok = (bz >= OBSTACLE_Z_MIN) & (bz <= OBSTACLE_Z_MAX)
    ahead = (bx > LIDAR_BODY_MIN_X) & (bx < stop_m) & (np.abs(by) < LIDAR_CONE_HALF_M)
    return int(np.count_nonzero(height_ok & ahead)) >= LIDAR_OBSTACLE_MIN_POINTS


def _drop_known_walls(
    pts: np.ndarray, known_wall: WallFilter | None
) -> np.ndarray:
    """Remove returns that land on saved map walls (planner already cleared them)."""
    if known_wall is None:
        return pts
    keep = ~known_wall(pts)
    return pts[keep]


def lidar_blocked_ahead(
    points: np.ndarray | None,
    pose: dict[str, Any] | None = None,
    *,
    stop_m: float = OBSTACLE_STOP_M,
    known_wall: WallFilter | None = None,
) -> bool:
    """True when lidar sees an obstacle in the forward body-frame cone."""
    if points is None or len(points) == 0 or pose is None:
        return False

    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 3:
        return False

    pts = _drop_known_walls(pts, known_wall)
    if len(pts) == 0:
        return False

    bx, by, bz = _body_xy_from_lidar(pts, pose)
    return _lidar_cone_blocked(bx, by, bz, stop_m=stop_m)


def lidar_blocked_toward(
    points: np.ndarray | None,
    pose: dict[str, Any] | None,
    target_x: float,
    target_y: float,
    *,
    stop_m: float = OBSTACLE_STOP_M,
    known_wall: WallFilter | None = None,
) -> bool:
    """True when lidar sees an obstacle along the line toward the steering target."""
    if points is None or len(points) == 0 or pose is None:
        return False

    px = float(pose["x"])
    py = float(pose["y"])
    if math.hypot(target_x - px, target_y - py) < 0.20:
        return False

    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 3:
        return False

    pts = _drop_known_walls(pts, known_wall)
    if len(pts) == 0:
        return False

    bx, by, bz = _body_xy_from_lidar(pts, pose)
    yaw = float(pose.get("yaw") or 0.0)
    heading = math.atan2(target_y - py, target_x - px)
    delta = angle_diff(yaw, heading)
    c = math.cos(delta)
    s = math.sin(delta)
    tx = c * bx + s * by
    ty = -s * bx + c * by
    return _lidar_cone_blocked(tx, ty, bz, stop_m=stop_m)


def _log_cone_block(
    points: np.ndarray | None,
    pose: dict[str, Any],
    target_x: float,
    target_y: float,
    stop_m: float,
    which: str,
    known_wall: WallFilter | None = None,
) -> None:
    """Throttled diagnostic: what the obstacle cone actually sees when it blocks."""
    global _last_obstacle_log_t
    now = time.time()
    if now - _last_obstacle_log_t < 1.0:
        return
    _last_obstacle_log_t = now
    try:
        pts = np.asarray(points, dtype=np.float64)
        wall = known_wall(pts) if known_wall is not None else np.zeros(len(pts), dtype=bool)
        bx, by, bz = _body_xy_from_lidar(pts, pose)
        px = float(pose.get("x") or 0.0)
        py = float(pose.get("y") or 0.0)
        yaw = float(pose.get("yaw") or 0.0)
        offset = angle_diff(yaw, math.atan2(target_y - py, target_x - px))
        # Report the cone that actually fired: "toward" rotates into the
        # target direction exactly like lidar_blocked_toward.
        if which == "toward":
            c, s = math.cos(offset), math.sin(offset)
            fx = c * bx + s * by
            fy = -s * bx + c * by
        else:
            fx, fy = bx, by
        height_ok = (bz >= OBSTACLE_Z_MIN) & (bz <= OBSTACLE_Z_MAX)
        in_cone = height_ok & (fx > LIDAR_BODY_MIN_X) & (fx < stop_m) & (np.abs(fy) < LIDAR_CONE_HALF_M)
        kept = in_cone & ~wall
        n = int(np.count_nonzero(in_cone))
        nk = int(np.count_nonzero(kept))
        nearest = float(np.min(fx[kept])) if nk else float("nan")
        lateral = float(np.median(fy[kept])) if nk else float("nan")
        _obstacle_log.warning(
            "BLOCK via %s: cone_pts=%d wall_pts=%d kept=%d nearest_x=%.2fm lateral=%.2fm "
            "stop_m=%.2f total_pts=%d pose_age=%.2f yaw_age=%.2f target_offset=%.0fdeg",
            which, n, int(np.count_nonzero(in_cone & wall)), nk, nearest, lateral,
            stop_m, len(pts),
            float(pose.get("age") or -1.0), float(pose.get("yaw_age") or -1.0),
            math.degrees(offset),
        )
    except Exception:
        pass


def is_tracking_blocked(
    pose: dict[str, Any],
    target_x: float,
    target_y: float,
    vx: float,
    vyaw: float,
    get_pose: PoseGetter | None,
    get_lidar: LidarGetter | None,
    *,
    stop_m: float = OBSTACLE_STOP_M,
    known_wall: WallFilter | None = None,
) -> bool:
    """Secondary lidar check — True when a tall obstacle is in the forward cone."""
    if abs(vx) < FORWARD_BLOCK_MIN_VX:
        return False

    if get_lidar is None:
        return False

    p = get_pose() if get_pose is not None else pose
    if p is None:
        return False

    points = get_lidar()
    if lidar_blocked_ahead(points, p, stop_m=stop_m, known_wall=known_wall):
        _log_cone_block(points, p, target_x, target_y, stop_m, "ahead", known_wall)
        return True
    if lidar_blocked_toward(points, p, target_x, target_y, stop_m=stop_m, known_wall=known_wall):
        _log_cone_block(points, p, target_x, target_y, stop_m, "toward", known_wall)
        return True
    return False


def is_segment_blocked(
    segment: NavSegment,
    get_pose: PoseGetter | None,
    get_lidar: LidarGetter | None,
    known_wall: WallFilter | None = None,
) -> bool:
    if segment.kind == "forward" and get_lidar is not None:
        pose = get_pose() if get_pose is not None else None
        if lidar_blocked_ahead(get_lidar(), pose, known_wall=known_wall):
            return True
    return False


def is_path_clear_ahead(
    get_pose: PoseGetter | None,
    get_lidar: LidarGetter | None,
    *,
    segment: NavSegment | None = None,
    known_wall: WallFilter | None = None,
) -> bool:
    """Hysteresis helper — require extra clearance before resuming after a pause."""
    seg = segment or NavSegment("forward", 1.0)
    if seg.kind == "forward" and get_lidar is not None:
        pose = get_pose() if get_pose is not None else None
        if lidar_blocked_ahead(get_lidar(), pose, stop_m=OBSTACLE_CLEAR_M, known_wall=known_wall):
            return False
    return True


def _lookahead_index(
    points: list[tuple[float, float]],
    wp_idx: int,
    pose: dict[str, Any],
) -> int:
    """Steer toward the furthest path point within lookahead distance."""
    px = float(pose["x"])
    py = float(pose["y"])
    target_idx = min(wp_idx, len(points) - 1)
    for i in range(wp_idx, len(points)):
        tx, ty = points[i]
        d = dist_xy(px, py, tx, ty)
        if d <= LOOKAHEAD_M or i == wp_idx:
            target_idx = i
        else:
            break
    return target_idx


def _skip_reached_waypoints(
    points: list[tuple[float, float]],
    wp_idx: int,
    pose: dict[str, Any],
) -> int:
    """Advance index past waypoints the robot has already reached."""
    px = float(pose["x"])
    py = float(pose["y"])
    while wp_idx < len(points) - 1:
        tx, ty = points[wp_idx]
        if dist_xy(px, py, tx, ty) < WAYPOINT_REACHED_M:
            wp_idx += 1
        else:
            break
    return wp_idx


def sport_move_no_reply(pub, vx: float, vy: float, vyaw: float) -> None:
    """Direct MCF Move on rt/api/sport/request — bypasses obstacle avoidance (teleop)."""
    generated_id = int(time.time() * 1000) % 2147483648 + random.randint(0, 1000)
    pub.publish_without_callback(
        RTC_TOPIC["SPORT_MOD"],
        {
            "header": {
                "identity": {"id": generated_id, "api_id": SPORT_CMD_MCF["Move"]},
                "policy": {"priority": 0, "noreply": True},
            },
            "parameter": json.dumps({"x": vx, "y": vy, "z": vyaw}),
            "binary": [],
        },
    )


def drive_no_reply(pub, vx: float, vy: float, vyaw: float) -> None:
    """Drive via the avoidance-guarded joystick channel (rt/wirelesscontroller).

    The Go2 firmware filters these commands and halts before obstacles — including
    the near-front zone the L1 lidar cannot see. Velocity intent is mapped to a
    normalized joystick: ly=forward, lx=strafe, rx=yaw. The yaw axis is inverted on
    this unit (commanding +rx turns clockwise), so positive vyaw (CCW) maps to -rx.
    """
    ly = max(-1.0, min(1.0, vx / WIRELESS_VX_FULL))
    lx = max(-1.0, min(1.0, -vy / WIRELESS_VY_FULL))
    rx = max(-1.0, min(1.0, -vyaw / WIRELESS_VYAW_FULL))
    pub.publish_without_callback(
        RTC_TOPIC["WIRELESS_CONTROLLER"],
        {"lx": lx, "ly": ly, "rx": rx, "ry": 0.0, "keys": 0},
    )


def drive_stop_no_reply(pub) -> None:
    """Zero the joystick to halt while staying in the avoidance-guarded channel."""
    pub.publish_without_callback(
        RTC_TOPIC["WIRELESS_CONTROLLER"],
        {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "keys": 0},
    )


async def enable_obstacle_avoidance(pub) -> None:
    """Turn on firmware avoidance and route motion through the joystick channel."""
    try:
        await pub.publish_request_new(
            RTC_TOPIC["OBSTACLES_AVOID"],
            {"api_id": OBSTACLES_AVOID_API["SWITCH_SET"], "parameter": {"enable": True}},
        )
        await pub.publish_request_new(
            RTC_TOPIC["OBSTACLES_AVOID"],
            {
                "api_id": OBSTACLES_AVOID_API["USE_REMOTE_COMMAND_FROM_API"],
                "parameter": {"is_remote_commands_from_api": False},
            },
        )
    except Exception:
        pass


async def sport_stop(pub) -> None:
    try:
        await pub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD_MCF["StopMove"]},
        )
    except Exception:
        pass


async def halt_robot(pub) -> None:
    await sport_stop(pub)
    try:
        await pub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD_MCF["BalanceStand"]},
        )
    except Exception:
        pass


async def _prepare_for_nav(pub) -> None:
    try:
        await pub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD_MCF["BalanceStand"]},
        )
        await enable_obstacle_avoidance(pub)
        await asyncio.sleep(0.3)
    except Exception:
        pass


async def _wait_until_clear(
    pub,
    segment: NavSegment,
    should_continue: ShouldContinue,
    get_pose: PoseGetter | None,
    get_lidar: LidarGetter | None,
    on_phase: OnNavPhase | None,
    obstacle_waits: list[int] | None = None,
    known_wall: WallFilter | None = None,
) -> bool:
    """Pause movement until the lidar cone reports a clear path."""
    drive_stop_no_reply(pub)
    if on_phase:
        on_phase("paused_obstacle")
    if obstacle_waits is not None:
        obstacle_waits[0] += 1

    clear_since: float | None = None
    wait_started = time.time()
    try:
        while should_continue():
            if time.time() - wait_started >= OBSTACLE_PAUSE_MAX_S:
                return False
            if is_path_clear_ahead(get_pose, get_lidar, segment=segment, known_wall=known_wall):
                if clear_since is None:
                    clear_since = time.time()
                elif time.time() - clear_since >= OBSTACLE_CLEAR_S:
                    return True
            else:
                clear_since = None
            await asyncio.sleep(OBSTACLE_WAIT_POLL_S)
        return False
    finally:
        if on_phase:
            on_phase("running")


def _tracking_velocity(
    pose: dict[str, Any],
    target_x: float,
    target_y: float,
    forward_speed: float = FORWARD_SPEED,
) -> tuple[float, float, float]:
    """Blend forward speed and yaw rate toward a lookahead point."""
    px = float(pose["x"])
    py = float(pose["y"])
    yaw = float(pose.get("yaw") or 0.0)

    dx = target_x - px
    dy = target_y - py
    dist = math.hypot(dx, dy)
    if dist < 1e-4:
        return 0.0, 0.0, 0.0

    target_yaw = math.atan2(dy, dx)
    yaw_err = angle_diff(yaw, target_yaw)

    # Large heading errors are handled by the loop's turn-in-place hysteresis;
    # here we only blend forward speed with a yaw correction for the aligned case.
    speed_scale = min(1.0, dist / SLOW_DIST_M) if dist < SLOW_DIST_M else 1.0
    align = max(0.35, math.cos(yaw_err))
    vx = forward_speed * speed_scale * align
    if abs(yaw_err) > 0.55:
        vx *= 0.8

    vyaw = max(-TURN_SPEED, min(TURN_SPEED, KP_YAW * yaw_err))
    return vx, 0.0, vyaw


async def _rotate_to_yaw(
    pub,
    get_pose: PoseGetter,
    should_continue: ShouldContinue,
    target_yaw: float,
    *,
    get_lidar: LidarGetter | None = None,
    on_phase: OnNavPhase | None = None,
    obstacle_waits: list[int] | None = None,
    tolerance_rad: float = 0.12,
    known_wall: WallFilter | None = None,
) -> bool:
    """Final heading alignment without discrete stop between legs."""
    segment = NavSegment("turn", 0.0)
    interval = 1.0 / MOVE_HZ
    stuck_since = time.time()
    # In-place turning keys off the IMU yaw (fresher than odom), so freshness is
    # tracked on yaw_stamp/yaw_age rather than the position stamp.
    last_vyaw: float | None = None
    last_yaw_stamp: float | None = None
    yaw_stale_since: float | None = None

    while should_continue():
        pose = get_pose()
        if pose is None or pose.get("yaw") is None:
            await asyncio.sleep(interval)
            continue

        # Stop turning if the heading estimate goes stale (no fresh feedback).
        yaw_age = pose.get("yaw_age")
        if yaw_age is not None and yaw_age > POSE_MAX_AGE_S:
            if yaw_stale_since is None:
                yaw_stale_since = time.time()
            drive_stop_no_reply(pub)
            last_vyaw = None
            await asyncio.sleep(interval)
            continue
        if yaw_stale_since is not None:
            stuck_since += time.time() - yaw_stale_since
            yaw_stale_since = None

        yaw_err = angle_diff(float(pose["yaw"]), target_yaw)
        if abs(yaw_err) <= tolerance_rad:
            return True

        if time.time() - stuck_since > STUCK_TIMEOUT_S:
            return False

        if is_segment_blocked(segment, get_pose, get_lidar, known_wall):
            if not await _wait_until_clear(
                pub,
                segment,
                should_continue,
                get_pose,
                get_lidar,
                on_phase,
                obstacle_waits,
                known_wall,
            ):
                return False
            last_vyaw = None
            await asyncio.sleep(interval)
            continue

        # Hold the last yaw rate until a new heading sample lands, so the turn
        # rate isn't re-commanded faster than feedback can correct it.
        yaw_stamp = pose.get("yaw_stamp")
        if last_vyaw is not None and yaw_stamp is not None and yaw_stamp == last_yaw_stamp:
            drive_no_reply(pub, 0.0, 0.0, last_vyaw)
            await asyncio.sleep(interval)
            continue
        last_yaw_stamp = yaw_stamp

        vyaw = max(-TURN_SPEED, min(TURN_SPEED, KP_YAW * yaw_err))
        drive_no_reply(pub, 0.0, 0.0, vyaw)
        last_vyaw = vyaw

        await asyncio.sleep(interval)

    return False


async def follow_waypoints_tracking(
    pub,
    get_pose: PoseGetter,
    should_continue: ShouldContinue,
    points: list[tuple[float, float]],
    *,
    final_yaw: float | None = None,
    forward_speed: float = FORWARD_SPEED,
    get_lidar: LidarGetter | None = None,
    on_phase: OnNavPhase | None = None,
    on_progress: Callable[[int, int], None] | None = None,
    known_wall: WallFilter | None = None,
) -> dict[str, Any]:
    """Closed-loop path follower: continuous joystick commands with live pose feedback."""
    if not points:
        return {
            "ok": False,
            "mode": "tracking",
            "status": "empty",
            "completed": 0,
            "obstacle_waits": 0,
        }

    pose = get_pose()
    if pose is None:
        return {
            "ok": False,
            "mode": "tracking",
            "status": "no_pose",
            "completed": 0,
            "obstacle_waits": 0,
        }

    await _prepare_for_nav(pub)
    obstacle_waits = [0]
    interval = 1.0 / MOVE_HZ
    total = len(points)
    wp_idx = 0
    wp_enter_t = time.time()
    progress_best_m = float("inf")
    progress_since = time.time()
    paused_for_obstacle = False
    # Feedback-timing state: hold the last setpoint between pose updates, and
    # stop entirely while localization is stale (see the guards in the loop).
    last_cmd: tuple[float, float, float] | None = None
    last_pos_stamp: float | None = None
    pose_stale_since: float | None = None
    paused_for_localization = False
    # Turn-in-place hysteresis: once a big heading error starts an in-place turn,
    # keep rotating until aligned within TURN_RESUME_RAD before driving off, so we
    # never give up a turn half-done and head off badly misaligned.
    aligning = False

    if on_progress:
        on_progress(0, total)

    try:
        while should_continue() and wp_idx < total:
            pose = get_pose()
            if pose is None:
                await asyncio.sleep(interval)
                continue

            # Freshness source: a pure in-place turn doesn't translate the base,
            # so the *position* estimate freezes (stamp/age stall) while IMU yaw
            # keeps updating. Gating on position there would stall the dog
            # mid-turn, so detect the turn-in-place intent up front and gate on
            # yaw freshness while rotating, position freshness while translating.
            tgt_idx = _lookahead_index(points, wp_idx, pose)
            heading = math.atan2(
                float(points[tgt_idx][1]) - float(pose["y"]),
                float(points[tgt_idx][0]) - float(pose["x"]),
            )
            turning_in_place = abs(angle_diff(float(pose.get("yaw") or 0.0), heading)) >= TURN_IN_PLACE_RAD
            fresh_age = pose.get("yaw_age") if turning_in_place else pose.get("age")
            fresh_stamp = pose.get("yaw_stamp") if turning_in_place else pose.get("stamp")

            # Stale-localization guard: steering on a frozen pose makes the dog
            # orbit its target, so when feedback can't keep up we stop and wait
            # for a fresh sample. The pause must not count against the
            # stuck/progress timers, so we offset them by the frozen duration.
            age = fresh_age
            if age is not None and age > POSE_MAX_AGE_S:
                if pose_stale_since is None:
                    pose_stale_since = time.time()
                if not paused_for_localization:
                    paused_for_localization = True
                    if on_phase:
                        on_phase("paused_localization")
                drive_stop_no_reply(pub)
                last_cmd = (0.0, 0.0, 0.0)
                await asyncio.sleep(interval)
                continue
            if pose_stale_since is not None:
                frozen = time.time() - pose_stale_since
                wp_enter_t += frozen
                progress_since += frozen
                pose_stale_since = None
            if paused_for_localization:
                paused_for_localization = False
                if on_phase:
                    on_phase("running")

            # Command-on-new-pose: commands stream at move_hz but odom arrives
            # slower, so re-deriving a correction from the same pose every tick
            # drives a latency limit cycle. Hold the last setpoint until a new
            # localization sample actually lands (yaw sample while turning in
            # place, position sample while translating — see fresh_stamp above).
            stamp = fresh_stamp
            if last_cmd is not None and stamp is not None and stamp == last_pos_stamp:
                drive_no_reply(pub, *last_cmd)
                await asyncio.sleep(interval)
                continue
            last_pos_stamp = stamp

            prev_idx = wp_idx
            wp_idx = _skip_reached_waypoints(points, wp_idx, pose)
            if wp_idx != prev_idx:
                wp_enter_t = time.time()
                progress_best_m = float("inf")
                progress_since = time.time()
                if on_progress:
                    on_progress(wp_idx, total)

            if wp_idx >= total:
                break

            target_idx = _lookahead_index(points, wp_idx, pose)
            tx, ty = points[target_idx]
            px = float(pose["x"])
            py = float(pose["y"])
            progress_m = dist_xy(px, py, float(points[wp_idx][0]), float(points[wp_idx][1]))

            if progress_m + 1e-3 < progress_best_m:
                progress_best_m = progress_m
                progress_since = time.time()
            elif time.time() - progress_since >= STUCK_RECOVER_S:
                if progress_m > WAYPOINT_REACHED_M and wp_idx < total - 1:
                    wp_idx += 1
                    wp_enter_t = time.time()
                    progress_best_m = float("inf")
                    progress_since = time.time()
                    if on_progress:
                        on_progress(wp_idx, total)
                    continue

            if progress_m < WAYPOINT_REACHED_M and wp_idx >= total - 1 and target_idx >= total - 1:
                wp_idx = total
                if on_progress:
                    on_progress(total, total)
                break

            if time.time() - wp_enter_t > STUCK_TIMEOUT_S:
                drive_stop_no_reply(pub)
                await sport_stop(pub)
                return {
                    "ok": False,
                    "mode": "tracking",
                    "status": "stuck",
                    "completed": wp_idx,
                    "failed_at": wp_idx,
                    "obstacle_waits": obstacle_waits[0],
                }

            # Turn-until-aligned (hysteresis): enter in-place turning at a big
            # heading error and stay in it until aligned within TURN_RESUME_RAD,
            # so a turn always completes instead of stopping half-done. Rotation
            # commands stream every tick, so the turn naturally "retries" until
            # the heading converges.
            heading_to_target = math.atan2(ty - py, tx - px)
            yaw_err = angle_diff(float(pose.get("yaw") or 0.0), heading_to_target)
            if abs(yaw_err) >= TURN_IN_PLACE_RAD:
                aligning = True
            elif abs(yaw_err) <= TURN_RESUME_RAD:
                aligning = False

            if aligning:
                vyaw = max(-TURN_SPEED, min(TURN_SPEED, KP_YAW * yaw_err * 1.15))
                drive_no_reply(pub, 0.0, 0.0, vyaw)
                last_cmd = (0.0, 0.0, vyaw)
                if on_phase:
                    on_phase("running")
                await asyncio.sleep(interval)
                continue

            vx, vy, vyaw = _tracking_velocity(pose, tx, ty, forward_speed)

            # Final-approach relaxation: a goal placed near a wall would otherwise
            # keep the stop cone (stop_m) latched on that wall and the dog could
            # never close the last metre. On the last waypoint, only react to
            # obstacles nearer than the goal itself (minus a small margin); the
            # planner already proved the goal reachable.
            stop_m = OBSTACLE_STOP_M
            if wp_idx >= total - 1:
                dist_goal = dist_xy(px, py, float(points[-1][0]), float(points[-1][1]))
                stop_m = min(OBSTACLE_STOP_M, max(LIDAR_BODY_MIN_X, dist_goal - FINAL_APPROACH_MARGIN_M))

            if is_tracking_blocked(
                pose, tx, ty, vx, vyaw, get_pose, get_lidar, stop_m=stop_m, known_wall=known_wall
            ):
                if not paused_for_obstacle:
                    paused_for_obstacle = True
                    obstacle_waits[0] += 1
                drive_stop_no_reply(pub)
                last_cmd = (0.0, 0.0, 0.0)
                if on_phase:
                    on_phase("paused_obstacle")
                await asyncio.sleep(interval)
                continue

            paused_for_obstacle = False
            if on_phase:
                on_phase("running")
            drive_no_reply(pub, vx, vy, vyaw)
            last_cmd = (vx, vy, vyaw)

            await asyncio.sleep(interval)

        if final_yaw is not None and should_continue():
            ok = await _rotate_to_yaw(
                pub,
                get_pose,
                should_continue,
                final_yaw,
                get_lidar=get_lidar,
                on_phase=on_phase,
                obstacle_waits=obstacle_waits,
                known_wall=known_wall,
            )
            if not ok:
                drive_stop_no_reply(pub)
                await sport_stop(pub)
                return {
                    "ok": False,
                    "mode": "tracking",
                    "status": "cancelled",
                    "completed": wp_idx,
                    "obstacle_waits": obstacle_waits[0],
                }

        drive_stop_no_reply(pub)
        await sport_stop(pub)
        return {
            "ok": True,
            "mode": "tracking",
            "status": "completed",
            "completed": total,
            "obstacle_waits": obstacle_waits[0],
        }
    except Exception:
        await sport_stop(pub)
        raise


async def follow_path_sport(
    pub,
    get_pose: PoseGetter,
    should_continue: ShouldContinue,
    points: list[tuple[float, float]],
    *,
    forward_speed: float = FORWARD_SPEED,
    get_lidar: LidarGetter | None = None,
    on_phase: OnNavPhase | None = None,
    on_progress: Callable[[int, int], None] | None = None,
    known_wall: WallFilter | None = None,
) -> dict[str, Any]:
    return await follow_waypoints_tracking(
        pub,
        get_pose,
        should_continue,
        points,
        forward_speed=forward_speed,
        get_lidar=get_lidar,
        on_phase=on_phase,
        on_progress=on_progress,
        known_wall=known_wall,
    )
