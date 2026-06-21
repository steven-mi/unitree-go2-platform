"""Live Go2 WebRTC connection with optional session recording."""

from __future__ import annotations

import asyncio
import json
import os
import threading
import time
from bisect import bisect_left
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import numpy as np

from config import (
    FLOOR_PLAN_REV,
    configured_aes_128_key,
    configured_nav_speed,
    configured_robot_ip,
    recordings_root,
    scans_root,
)
from domain.models import IndexEntry
from floorplan.builder import (
    FloorPlanBuilder,
    floor_plan_to_api,
)
from parsing.rpc import summarize_rpc
from parsing.topics import (
    parse_audio,
    parse_battery,
    parse_battery_state,
    parse_lidar_state,
    parse_pose,
    parse_sport,
    parse_sport_pose,
    parse_system,
    parse_uwb,
    parse_velocity,
)
from live.navigation import (
    dist_xy,
    follow_path_sport,
    halt_robot,
    sport_stop,
)
from route_planning.planner import make_known_wall_filter
from scan import store as scan_store
from live.teleop import clamp_drive, send_drive
from recording.recorder import (
    FullSessionRecorder,
    capture_photo,
    repair_session,
    run_rpc_probes,
)
from replay.store import read_session_tags
from scan.store import GRID_FILENAME, LATEST_SCAN_ID
from recording.util import (
    LIDAR_NPZ_LABELS,
    SKIP_SUBSCRIBE,
    extract_lidar_points,
    robot_stamp_sec,
    strip_heavy_fields,
)
from unitree_webrtc_connect.constants import (
    RTC_TOPIC,
    SPORT_CMD_MCF,
    WebRTCConnectionMethod,
)
from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection

RECORDINGS_ROOT = recordings_root()
CLIENT_IDLE_TIMEOUT_S = 120.0
LINK_DEAD_GRACE_S = 6.0
PEER_DOWN_GRACE_S = 8.0
TELEOP_HZ = 100.0


@dataclass
class LiveStatus:
    state: str  # idle | connecting | connected | error
    connected: bool
    recording: bool
    session_id: str | None
    error: str | None
    robot_ip: str
    duration_s: float
    lidar_count: int
    video_count: int


@dataclass
class NavigationState:
    active: bool = False
    ok: bool | None = None
    status: str = "idle"
    completed: int = 0
    total: int = 0
    failed_at: int | None = None
    mode: str = "segments"
    error: str | None = None
    paused_obstacle: bool = False
    paused_localization: bool = False


class LiveManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True, name="live-loop")
        self._thread.start()

        self.state = "idle"
        self.error: str | None = None
        self._robot_ip = configured_robot_ip()
        self._conn: UnitreeWebRTCConnection | None = None
        self._stop = asyncio.Event()
        self._connect_task: asyncio.Future | None = None
        self._connecting_since = 0.0
        self._last_client_at = 0.0

        self._start_t = 0.0
        self._recorder: FullSessionRecorder | None = None
        self._session_id: str | None = None

        self.video: list[IndexEntry] = []
        self.lidar: list[IndexEntry] = []
        self.odom: list[IndexEntry] = []
        self.sport: list[IndexEntry] = []
        self.battery: list[IndexEntry] = []
        self.ulidar_state: list[IndexEntry] = []
        self.uwb: list[IndexEntry] = []
        self.multiple_state: list[IndexEntry] = []
        self.audio_hub: list[IndexEntry] = []
        self._lidar_points: dict[int, np.ndarray] = {}
        self._latest_video_jpg: bytes | None = None
        self._rpc: dict[str, Any] = {}

        self._floor_builder: FloorPlanBuilder | None = None
        self._floor_plan_rev = 0
        self._floor_lidar_i = 0
        self._floor_odom_i = 0
        self._scan_lidar_start = 0
        self._scan_odom_start = 0
        self._go_to_gen = 0
        self._nav_gen = 0
        self._nav_state = NavigationState()
        self._nav_lock = threading.Lock()
        self._link_unhealthy_since: float | None = None
        self._disconnect_reason: str | None = None

        self._drive_lock = threading.Lock()
        self._drive_vx = 0.0
        self._drive_vy = 0.0
        self._drive_vyaw = 0.0
        self._teleop_was_moving = False

    def _zero_drive_target(self) -> None:
        with self._drive_lock:
            self._drive_vx = 0.0
            self._drive_vy = 0.0
            self._drive_vyaw = 0.0

    def current_pose(self) -> dict[str, Any] | None:
        with self._lock:
            if self.state != "connected":
                return None
            t = self._elapsed_s()
            odom = self._nearest(self.odom, t)
            sport = self._nearest(self.sport, t)
        now = time.time()
        odom_pose = parse_pose(odom.payload if odom else None)
        sport_pose = parse_sport_pose(sport.payload if sport else None)

        # Position comes from ROBOTODOM (lidar-inertial localization); fall back
        # to the sport state only when odom is unavailable.
        if odom_pose is not None:
            pose = odom_pose
            pos_stamp = odom.recv_t if odom else None
        elif sport_pose is not None:
            pose = sport_pose
            pos_stamp = sport.recv_t if sport else None
        else:
            return None

        # IMU yaw (sport) tracks in-place turns; ROBOTODOM yaw often lags, so
        # prefer the IMU heading when present and track which source it came from
        # (its freshness drives in-place rotation).
        yaw_stamp = pos_stamp
        if sport_pose is not None and sport_pose.get("yaw") is not None:
            pose = {**pose, "yaw": sport_pose["yaw"]}
            yaw_stamp = sport.recv_t if sport else None

        # recv_t is wall-clock (time.time at ingest), so age is real seconds of
        # localization lag. The control loop uses this to stop on stale pose and
        # to only re-steer when a new sample actually arrives.
        pose["stamp"] = pos_stamp
        pose["age"] = (now - pos_stamp) if pos_stamp is not None else None
        pose["yaw_stamp"] = yaw_stamp
        pose["yaw_age"] = (now - yaw_stamp) if yaw_stamp is not None else None
        return pose

    def latest_lidar_points(self) -> np.ndarray | None:
        with self._lock:
            if not self.lidar:
                return None
            seq = self.lidar[-1].payload.get("seq")
            cached = self._lidar_points.get(seq) if seq is not None else None
            row = self.lidar[-1].payload
            recorder = self._recorder
        if cached is not None:
            return cached
        if recorder is not None:
            fname = row.get("file")
            if fname:
                path = recorder.root / "lidar" / fname
                if path.exists():
                    with np.load(path) as data:
                        return np.asarray(data["points"], dtype=np.float32)
        return None

    def _run(self, coro, timeout: float = 60.0):
        fut = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return fut.result(timeout=timeout)

    async def _teleop_loop(self) -> None:
        """Stream Move commands at a fixed rate so teleop stays smooth."""
        interval = 1.0 / TELEOP_HZ
        try:
            while not self._stop.is_set():
                with self._lock:
                    conn = self._conn
                    connected = self.state == "connected"
                if not connected:
                    await asyncio.sleep(interval)
                    continue
                if conn is None or not conn.isConnected:
                    await asyncio.sleep(interval)
                    continue

                with self._drive_lock:
                    vx, vy, vyaw = self._drive_vx, self._drive_vy, self._drive_vyaw

                moving = abs(vx) + abs(vy) + abs(vyaw) > 0.001
                with self._nav_lock:
                    nav_active = self._nav_state.active

                # Manual teleop overrides path navigation (drive() cancels nav first).
                if moving or not nav_active:
                    pub = conn.datachannel.pub_sub
                    if moving:
                        send_drive(pub, vx, vy, vyaw)
                        self._teleop_was_moving = True
                    elif self._teleop_was_moving:
                        send_drive(pub, 0.0, 0.0, 0.0)
                        self._teleop_was_moving = False

                await asyncio.sleep(interval)
        finally:
            with self._lock:
                conn = self._conn
            if conn is not None and conn.isConnected:
                try:
                    send_drive(conn.datachannel.pub_sub, 0.0, 0.0, 0.0)
                except Exception:
                    pass
            self._teleop_was_moving = False
            self._zero_drive_target()

    def touch_client(self) -> None:
        with self._lock:
            self._last_client_at = time.time()

    def _link_healthy_unlocked(self) -> bool:
        """True when WebRTC peer is up (caller must hold self._lock)."""
        if self.state != "connected":
            return False
        conn = self._conn
        if conn is None or not conn.isConnected:
            return False
        pc = getattr(conn, "pc", None)
        if pc is not None and pc.connectionState in ("failed", "closed"):
            return False
        return True

    def _mark_link_unhealthy(self) -> None:
        """Start or continue the grace timer before declaring the session dead."""
        now = time.time()
        if self._link_unhealthy_since is None:
            self._link_unhealthy_since = now

    def _clear_link_unhealthy(self) -> None:
        self._link_unhealthy_since = None

    def _link_unhealthy_long_enough(self) -> bool:
        if self._link_unhealthy_since is None:
            return False
        return (time.time() - self._link_unhealthy_since) >= LINK_DEAD_GRACE_S

    def _reconcile_dead_link(self) -> None:
        """Mark state idle after the peer stays down past a short grace window."""
        should_stop = False
        with self._lock:
            if self.state != "connected":
                return
            if self._link_healthy_unlocked():
                self._clear_link_unhealthy()
                return
            self._mark_link_unhealthy()
            if not self._link_unhealthy_long_enough():
                return
            reason = self._disconnect_reason or "peer_lost"
            self.state = "idle"
            self.error = self.error or f"Connection lost ({reason.replace('_', ' ')})"
            task = self._connect_task
            should_stop = task is not None and not task.done()
        if should_stop:
            self._stop.set()

    def _elapsed_s(self) -> float:
        if not self._start_t:
            return 0.0
        return max(0.0, time.time() - self._start_t)

    def _snapshot_status(self) -> LiveStatus:
        elapsed = self._elapsed_s()
        return LiveStatus(
            state=self.state,
            connected=self._link_healthy_unlocked(),
            recording=self._recorder is not None,
            session_id=self._session_id,
            error=self.error,
            robot_ip=self._robot_ip,
            duration_s=round(elapsed, 2),
            lidar_count=len(self.lidar),
            video_count=len(self.video),
        )

    def status(self) -> LiveStatus:
        self.touch_client()
        self._reconcile_dead_link()
        with self._lock:
            return self._snapshot_status()

    def set_robot_ip(self, ip: str) -> None:
        cleaned = ip.strip()
        if not cleaned:
            return
        with self._lock:
            if self.state != "connected":
                self._robot_ip = cleaned

    def _reconcile_stale_state(self) -> None:
        stale = False
        with self._lock:
            task = self._connect_task
            task_done = task is None or task.done()
            if self.state == "connecting" and task_done:
                self.state = "idle"
                self.error = None
                self._connect_task = None
                self._connecting_since = 0.0
            elif (
                self.state == "connecting"
                and not task_done
                and self._connecting_since > 0
                and (time.time() - self._connecting_since) > 50.0
            ):
                stale = True
        if stale:
            self._force_idle()

    def _force_idle(self) -> None:
        try:
            self._run(self._stop_connection(), timeout=8.0)
        except Exception:
            with self._lock:
                task = self._connect_task
                if task is not None and not task.done():
                    self._loop.call_soon_threadsafe(task.cancel)
                self._connect_task = None
                self._conn = None
                self.state = "idle"
                self.error = None
                self._start_t = 0.0
                self._connecting_since = 0.0
                self._stop = asyncio.Event()

    def _tear_down(self) -> None:
        try:
            self._run(self._stop_connection(), timeout=20.0)
        except Exception:
            self._force_idle()
        time.sleep(0.35)

    def connect(self, ip: str | None = None) -> LiveStatus:
        if ip:
            cleaned = ip.strip()
            if cleaned:
                with self._lock:
                    if cleaned == self._robot_ip and self._link_healthy_unlocked():
                        return self._snapshot_status()
                    if cleaned != self._robot_ip:
                        self._robot_ip = cleaned

        self._reconcile_dead_link()
        self._reconcile_stale_state()

        with self._lock:
            if self._link_healthy_unlocked():
                return self._snapshot_status()

        self._tear_down()
        self._reconcile_stale_state()

        with self._lock:
            self.state = "connecting"
            self.error = None
            self._connecting_since = time.time()
        self._run(self._start_connection())
        deadline = time.time() + 45.0
        while time.time() < deadline:
            self._reconcile_stale_state()
            status = self.status()
            if status.state in ("connected", "error"):
                return status
            time.sleep(0.15)
        return self.status()

    def disconnect(self) -> LiveStatus:
        self._tear_down()
        return self.status()

    def request_stop(self) -> None:
        """Non-blocking stop signal for reload/SIGTERM before full teardown."""
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._stop.set)

    def shutdown(self) -> None:
        self.request_stop()
        self._tear_down()
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=3.0)

    def start_recording(self, name: str = "", note: str = "") -> LiveStatus:
        with self._lock:
            if self.state != "connected":
                raise RuntimeError("Connect to the robot before recording")
            if self._recorder is not None:
                return self._snapshot_status()
            conn = self._conn

            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            suffix = f"_{name}" if name else ""
            session_id = f"{stamp}{suffix}"
            root = RECORDINGS_ROOT / session_id
            self._recorder = FullSessionRecorder(root, note=note)
            self._session_id = session_id

        if conn is not None:
            self._run(self._bootstrap_recording(conn))
        return self.status()

    async def _bootstrap_recording(self, conn) -> None:
        recorder = self._recorder
        if recorder is None:
            return
        await run_rpc_probes(conn, recorder, 8.0)
        await capture_photo(conn, recorder, 8.0)

    def stop_recording(self) -> dict[str, Any]:
        with self._lock:
            if self._recorder is None:
                raise RuntimeError("Not recording")
            recorder = self._recorder
            session_id = self._session_id
            self._recorder = None
            self._session_id = None

        with recorder.lock:
            recorder._accepting = False

        with self._lock:
            ip = self._robot_ip
        elapsed = max(0.0, time.time() - recorder.counts.start_t)
        manifest = recorder.close({
            "robot_ip": ip,
            "session_id": session_id,
            "duration_s": round(elapsed, 2),
            "note": _read_note(recorder.root),
            "tags": read_session_tags(recorder.root),
            "interrupted": False,
            "video_fps": 5.0,
        })
        return {"session_id": session_id, "manifest": manifest.read_text(encoding="utf-8")}

    def stop_navigation(self) -> None:
        with self._lock:
            self._go_to_gen += 1
            conn = self._conn
            connected = self._link_healthy_unlocked()
        with self._nav_lock:
            if self._nav_state.active:
                self._nav_state.active = False
                self._nav_state.status = "cancelled"
                self._nav_state.paused_obstacle = False
                self._nav_state.paused_localization = False
                self._nav_state.error = "Navigation stopped"
        if conn is not None and connected and conn.isConnected:
            try:
                asyncio.run_coroutine_threadsafe(
                    self._halt_robot_async(conn),
                    self._loop,
                )
            except Exception:
                pass

    def _cancel_active_navigation(self) -> None:
        """Stop an in-flight route so a new Go can start immediately."""
        with self._nav_lock:
            if not self._nav_state.active:
                return
        self.stop_navigation()

    def _begin_nav_run(self, *, mode: str, total: int) -> int:
        with self._lock:
            self._go_to_gen += 1
            gen = self._go_to_gen
        with self._nav_lock:
            self._nav_gen = gen
            self._nav_state = NavigationState(
                active=True,
                ok=None,
                status="running",
                completed=0,
                total=total,
                mode=mode,
            )
        return gen

    def navigation_status(self) -> dict[str, Any]:
        self.touch_client()
        with self._nav_lock:
            return {
                "active": self._nav_state.active,
                "ok": self._nav_state.ok,
                "status": self._nav_state.status,
                "completed": self._nav_state.completed,
                "total": self._nav_state.total,
                "failed_at": self._nav_state.failed_at,
                "mode": self._nav_state.mode,
                "error": self._nav_state.error,
                "paused_obstacle": self._nav_state.paused_obstacle,
                "paused_localization": self._nav_state.paused_localization,
            }

    def _set_nav_phase(self, phase: str) -> None:
        with self._nav_lock:
            if phase == "paused_obstacle":
                self._nav_state.paused_obstacle = True
                self._nav_state.paused_localization = False
                self._nav_state.status = "paused_obstacle"
            elif phase == "paused_localization":
                self._nav_state.paused_localization = True
                self._nav_state.paused_obstacle = False
                self._nav_state.status = "paused_localization"
            else:
                self._nav_state.paused_obstacle = False
                self._nav_state.paused_localization = False
                if self._nav_state.active:
                    self._nav_state.status = "running"

    def _set_nav_progress(self, completed: int, total: int) -> None:
        self.touch_client()
        with self._nav_lock:
            self._nav_state.completed = completed
            self._nav_state.total = total

    def _finish_nav(self, result: dict[str, Any], gen: int) -> None:
        with self._nav_lock:
            if gen != self._nav_gen:
                return
            self._nav_state.active = False
            self._nav_state.ok = bool(result.get("ok"))
            self._nav_state.status = str(result.get("status") or "failed")
            self._nav_state.completed = int(result.get("completed") or 0)
            self._nav_state.failed_at = result.get("failed_at")
            self._nav_state.mode = str(result.get("mode") or "tracking")
            self._nav_state.paused_obstacle = False
            self._nav_state.paused_localization = False
            err = result.get("error")
            if isinstance(err, str) and err.strip():
                self._nav_state.error = err.strip()
            elif self._nav_state.status == "obstacle_timeout":
                self._nav_state.error = (
                    "Obstacle did not clear in time — use Stop, then try Go again"
                )
            elif self._nav_state.status == "stuck":
                self._nav_state.error = (
                    "Could not reach the next waypoint — check for obstacles or try Locate again"
                )
            elif self._nav_state.status == "cancelled":
                with self._lock:
                    healthy = self._link_healthy_unlocked()
                self._nav_state.error = (
                    "Connection lost during navigation — reconnect and try Go again"
                    if not healthy
                    else "Navigation stopped"
                )
            else:
                self._nav_state.error = None

    def _fail_nav(self, message: str, gen: int) -> None:
        with self._nav_lock:
            if gen != self._nav_gen:
                return
            self._nav_state.active = False
            self._nav_state.ok = False
            self._nav_state.status = "failed"
            self._nav_state.error = message

    async def _halt_robot_async(self, conn) -> None:
        await halt_robot(conn.datachannel.pub_sub)

    async def _stop_drive_async(self, conn) -> None:
        await sport_stop(conn.datachannel.pub_sub)

    async def _sport_command_async(
        self,
        conn,
        command: str,
        parameter: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        if command not in SPORT_CMD_MCF:
            raise ValueError(f"Unknown sport command: {command}")
        payload: dict[str, Any] = {"api_id": SPORT_CMD_MCF[command]}
        if parameter is not None:
            payload["parameter"] = parameter
        response = await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            payload,
        )
        code = response.get("data", {}).get("header", {}).get("status", {}).get("code", -1)
        return {"ok": code == 0, "code": code}

    def _require_conn(self):
        self._reconcile_dead_link()
        with self._lock:
            if not self._link_healthy_unlocked():
                raise RuntimeError("Connect to the robot first")
            conn = self._conn
        if conn is None:
            raise RuntimeError("Not connected to robot")
        return conn

    def drive(self, vx: float, vy: float, vyaw: float) -> None:
        self.touch_client()
        vx, vy, vyaw = clamp_drive(vx, vy, vyaw)
        moving = abs(vx) + abs(vy) + abs(vyaw) > 0.001
        if moving:
            with self._nav_lock:
                nav_active = self._nav_state.active
            if nav_active:
                self.stop_navigation()
        try:
            self._require_conn()
        except RuntimeError:
            if not moving:
                self._zero_drive_target()
            raise
        with self._drive_lock:
            self._drive_vx = vx
            self._drive_vy = vy
            self._drive_vyaw = vyaw

    def stop_drive(self) -> None:
        self.touch_client()
        self._zero_drive_target()

    def sport_command(self, command: str, parameter: dict[str, Any] | None = None) -> dict[str, Any]:
        conn = self._require_conn()
        return self._run(self._sport_command_async(conn, command, parameter), timeout=15.0)

    def follow_path(self, points: list[dict[str, float]], *, map_frame: bool = False) -> dict[str, Any]:
        """Start path following in the background; poll navigation_status() for progress."""
        self.touch_client()
        if not points:
            raise RuntimeError("Path is empty")

        if self.state != "connected":
            raise RuntimeError("Connect to the robot first")

        pose = self.current_pose()
        if pose is not None:
            trimmed = list(points)
            while len(trimmed) > 1:
                head = trimmed[0]
                if dist_xy(
                    float(pose["x"]),
                    float(pose["y"]),
                    float(head["x"]),
                    float(head["y"]),
                ) < 0.3:
                    trimmed.pop(0)
                else:
                    break
            points = trimmed

        if len(points) < 1:
            raise RuntimeError("Path is empty after trimming")

        self._cancel_active_navigation()
        total = len(points)
        gen = self._begin_nav_run(mode="tracking", total=total)

        def _done(fut: asyncio.Future) -> None:
            try:
                result = fut.result()
                self._finish_nav(result, gen)
            except Exception as exc:
                self._fail_nav(str(exc), gen)

        fut = asyncio.run_coroutine_threadsafe(
            self._follow_path_async(points, gen),
            self._loop,
        )
        fut.add_done_callback(_done)
        return {"ok": True, "started": True, "total": total, "replaced": True}

    def _build_known_wall_filter(self):
        """Wall classifier for the saved map so the safety cone ignores known walls.

        The route was planned against ``scans/latest`` (walls inflated by the robot
        footprint), so the driving cone only needs to stop for *unmapped* obstacles.
        Built once per run from the same floor plan + localization transform the
        planner used. Returns ``None`` (cone reacts to everything) if unavailable.
        """
        try:
            floorplan = scan_store.load_floorplan(LATEST_SCAN_ID)
        except FileNotFoundError:
            return None
        if "walls_b64" not in floorplan:
            return None
        try:
            align = scan_store.load_scan_meta(LATEST_SCAN_ID).get("map_alignment") or {}
        except FileNotFoundError:
            align = {}
        return make_known_wall_filter(
            floorplan,
            tx=float(align.get("tx") or 0.0),
            ty=float(align.get("ty") or 0.0),
            dyaw=float(align.get("dyaw") or 0.0),
        )

    async def _follow_path_async(
        self,
        points: list[dict[str, float]],
        gen: int,
    ) -> dict[str, Any]:
        with self._lock:
            conn = self._conn
        if conn is None or not conn.isConnected:
            raise RuntimeError("Not connected to robot")

        coords = [(float(p["x"]), float(p["y"])) for p in points]
        known_wall = self._build_known_wall_filter()

        def get_pose() -> dict[str, Any] | None:
            return self.current_pose()

        def should_continue() -> bool:
            self.touch_client()
            with self._lock:
                if gen != self._go_to_gen or self.state != "connected":
                    return False
            return True

        def on_progress(completed: int, total: int) -> None:
            self._set_nav_progress(completed, total)

        def on_phase(phase: str) -> None:
            self._set_nav_phase(phase)

        return await follow_path_sport(
            conn.datachannel.pub_sub,
            get_pose,
            should_continue,
            coords,
            forward_speed=configured_nav_speed(),
            get_lidar=self.latest_lidar_points,
            on_phase=on_phase,
            on_progress=on_progress,
            known_wall=known_wall,
        )

    def rel_t(self, recv_t: float) -> float:
        return max(0.0, recv_t - self._start_t)

    @property
    def duration(self) -> float:
        with self._lock:
            return self._elapsed_s()

    def _nearest(self, entries: list[IndexEntry], t: float) -> IndexEntry | None:
        if not entries:
            return None
        times = [self.rel_t(e.recv_t) for e in entries]
        i = bisect_left(times, t)
        if i == 0:
            return entries[0]
        if i >= len(entries):
            return entries[-1]
        before, after = entries[i - 1], entries[i]
        if abs(self.rel_t(after.recv_t) - t) < abs(t - self.rel_t(before.recv_t)):
            return after
        return before

    def frame_at(self, t: float | None = None) -> dict[str, Any]:
        self.touch_client()
        with self._lock:
            duration = max(0.01, self._elapsed_s())
            if t is None:
                t = duration
            t = max(0.0, min(float(t), duration))

            video = self._nearest(self.video, t)
            lidar = self._nearest(self.lidar, t)
            odom = self._nearest(self.odom, t)
            sport = self._nearest(self.sport, t)
            battery = self._nearest(self.battery, t)
            ulidar_state = self._nearest(self.ulidar_state, t)
            uwb = self._nearest(self.uwb, t)
            multiple_state = self._nearest(self.multiple_state, t)
            audio_hub = self._nearest(self.audio_hub, t)
            session_id = self._session_id
            recording = self._recorder is not None

            pose = parse_pose(odom.payload if odom else None)
            if pose is None and sport:
                pose = parse_sport_pose(sport.payload)

            battery_info = parse_battery_state(battery.payload if battery else None)
            out: dict[str, Any] = {
                "t": round(t, 3),
                "duration": round(duration, 3),
                "live": True,
                "recording": recording,
                "session_id": session_id,
                "video": None,
                "lidar": None,
                "pose": pose,
                "velocity": parse_velocity(sport.payload if sport else None),
                "battery_v": battery_info.get("voltage") if battery_info else parse_battery(battery.payload if battery else None),
                "sport": parse_sport(sport.payload if sport else None),
                "battery": battery_info,
                "lidar_state": parse_lidar_state(ulidar_state.payload if ulidar_state else None),
                "uwb": parse_uwb(uwb.payload if uwb else None),
                "system": parse_system(multiple_state.payload if multiple_state else None),
                "audio": parse_audio(audio_hub.payload if audio_hub else None),
            }

            if video:
                seq = video.payload.get("seq")
                out["video"] = {
                    "seq": seq,
                    "file": video.payload.get("file", "latest.jpg"),
                    "url": f"/api/live/video/latest.jpg?seq={seq}",
                }
            if lidar:
                seq = lidar.payload.get("seq")
                url = f"/api/live/lidar/{seq}"
                out["lidar"] = {
                    "seq": seq,
                    "file": lidar.payload.get("file"),
                    "point_count": lidar.payload.get("point_count"),
                    "url": url,
                }
            return out

    def session_detail(self) -> dict[str, Any]:
        with self._lock:
            rpc = dict(self._rpc)
            session_id = self._session_id or "live"
            duration = self._elapsed_s()
        return {
            "id": session_id,
            "duration": round(duration, 3),
            "rpc": rpc,
            "services": [],
        }

    def load_lidar_points_binary(self, seq: int, max_points: int = 0) -> bytes:
        points, _ = self._load_lidar_array(seq, max_points=max_points)
        return points.tobytes()

    def _load_lidar_array(self, seq: int, max_points: int = 8000) -> tuple[np.ndarray, dict[str, Any]]:
        with self._lock:
            row = next((e.payload for e in self.lidar if e.payload.get("seq") == seq), None)
            if row is None:
                raise FileNotFoundError(f"lidar seq {seq} not found")
            session_id = self._session_id
            recorder = self._recorder
            cached = self._lidar_points.get(seq)

        if cached is not None:
            points = cached
        elif recorder is not None and session_id:
            path = recorder.root / "lidar" / row["file"]
            with np.load(path) as data:
                points = np.asarray(data["points"], dtype=np.float32)
        else:
            raise FileNotFoundError(f"lidar seq {seq} not found")

        if max_points > 0 and len(points) > max_points:
            idx = np.linspace(0, len(points) - 1, max_points, dtype=int)
            points = points[idx]
        return points, row

    def latest_video_jpg(self) -> bytes | None:
        with self._lock:
            if self._latest_video_jpg:
                return self._latest_video_jpg
            if self._recorder and self.video:
                last = self.video[-1].payload.get("file")
                if last:
                    path = self._recorder.root / "video" / last
                    if path.exists():
                        return path.read_bytes()
            return None

    def save_floor_grid(self, dest: Path) -> bool:
        """Persist the live floor-plan evidence grid to ``dest/floor_grid.npz``.

        Snapshots the builder's accumulated occupancy grid (compact, ~KB) instead
        of every raw lidar frame, so a later connect can resume mapping instantly.
        """
        with self._lock:
            builder = self._floor_builder
            if builder is None:
                return False
            return builder.save_state(dest / GRID_FILENAME, revision=FLOOR_PLAN_REV)

    def load_latest_scan_session(self) -> bool:
        """Resume mapping from scans/latest by restoring its saved evidence grid.

        Returns ``False`` (start fresh) when there is no saved grid or it was
        built under different floor-plan parameters — the source recording still
        holds the raw frames if a full reprocess is ever wanted.
        """
        grid_file = scans_root() / LATEST_SCAN_ID / GRID_FILENAME
        if not grid_file.exists():
            return False
        try:
            builder, revision = FloorPlanBuilder.from_state(grid_file)
        except Exception:
            return False
        if revision != FLOOR_PLAN_REV or builder.score is None:
            return False

        with self._lock:
            self._floor_builder = builder
            self._floor_plan_rev = FLOOR_PLAN_REV
            # Only frames captured from now on feed the restored grid; its history
            # is already baked into the saved score grid.
            self._floor_lidar_i = 0
            self._floor_odom_i = 0
            self._scan_lidar_start = len(self.lidar)
            self._scan_odom_start = len(self.odom)

        return True

    def reset_scan_epoch(self) -> None:
        """Start a fresh scan epoch — only lidar/odom after this point feed the map."""
        with self._lock:
            self._scan_lidar_start = len(self.lidar)
            self._scan_odom_start = len(self.odom)
            self._floor_builder = None
            self._floor_lidar_i = 0
            self._floor_odom_i = 0

    def build_floor_plan(
        self,
        upto_t: float | None = None,
        *,
        robot_x: float | None = None,
        robot_y: float | None = None,
        lidar_seq: int | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            duration = max(0.01, self._elapsed_s())
            t_limit = duration if upto_t is None else max(0.0, min(float(upto_t), duration))
            builder = self._ensure_floor_builder()
            self._ingest_floor_plan(builder, t_limit)
            if lidar_seq is not None:
                self._ingest_lidar_seq(builder, lidar_seq, robot_x=robot_x, robot_y=robot_y)
            try:
                result = builder.build(robot_x=robot_x, robot_y=robot_y, crop=True)
            except ValueError:
                return _empty_floor_plan(upto_t=t_limit)
            return floor_plan_to_api(result, upto_t=t_limit)

    def _ensure_floor_builder(self) -> FloorPlanBuilder:
        if self._floor_builder is None or self._floor_plan_rev != FLOOR_PLAN_REV:
            self._floor_builder = FloorPlanBuilder()
            self._floor_plan_rev = FLOOR_PLAN_REV
            self._floor_lidar_i = 0
            self._floor_odom_i = 0
        return self._floor_builder

    def _ingest_floor_plan(self, builder: FloorPlanBuilder, data_limit: float) -> None:
        # Only fuse a lidar frame once an odom sample at or after its timestamp
        # has arrived. ROBOTODOM is point-in-time (never retroactively rewritten),
        # so the nearest-pose match is final the moment a later odom exists — and
        # would otherwise shift as more odom streams in. Deferring here gives the
        # live map the same settled pose per scan as the offline replay (whose
        # odom track is already complete), instead of baking in trailing-edge
        # localization lag. With no odom at all we cannot ray-cast, so fall back
        # to fusing immediately (occupied-only evidence).
        latest_odom_t = self.rel_t(self.odom[-1].recv_t) if self.odom else None

        abs_i = self._scan_lidar_start + self._floor_lidar_i
        while abs_i < len(self.lidar):
            entry = self.lidar[abs_i]
            t_scan = self.rel_t(entry.recv_t)
            if t_scan > data_limit:
                break
            if latest_odom_t is not None and t_scan > latest_odom_t:
                break
            row = entry.payload
            seq = row.get("seq")
            pts = self._lidar_points.get(seq)
            if pts is None and self._recorder is not None:
                path = self._recorder.root / "lidar" / row["file"]
                if path.exists():
                    with np.load(path, mmap_mode="r") as data:
                        pts = np.asarray(data["points"], dtype=np.float32)
            if pts is not None:
                odom = self._nearest(self.odom, t_scan)
                pose = parse_pose(odom.payload if odom else None)
                builder.ingest_points(
                    pts,
                    robot_x=pose["x"] if pose else None,
                    robot_y=pose["y"] if pose else None,
                )
            self._floor_lidar_i += 1
            abs_i += 1

        abs_odom = self._scan_odom_start + self._floor_odom_i
        while abs_odom < len(self.odom):
            entry = self.odom[abs_odom]
            if self.rel_t(entry.recv_t) > data_limit:
                break
            if self._floor_odom_i % 2 == 0:
                pose = parse_pose(entry.payload)
                if pose is not None:
                    builder.append_path(pose["x"], pose["y"])
            self._floor_odom_i += 1
            abs_odom += 1

    def _ingest_lidar_seq(
        self,
        builder: FloorPlanBuilder,
        seq: int,
        *,
        robot_x: float | None = None,
        robot_y: float | None = None,
    ) -> None:
        row = next((e.payload for e in self.lidar if e.payload.get("seq") == seq), None)
        if row is None:
            return
        entry = next((e for e in self.lidar if e.payload.get("seq") == seq), None)
        pts = self._lidar_points.get(seq)
        if pts is None and self._recorder is not None:
            path = self._recorder.root / "lidar" / row["file"]
            if path.exists():
                with np.load(path, mmap_mode="r") as data:
                    pts = np.asarray(data["points"], dtype=np.float32)
        if pts is None:
            return
        if entry is not None:
            t_scan = self.rel_t(entry.recv_t)
            odom = self._nearest(self.odom, t_scan)
            pose = parse_pose(odom.payload if odom else None)
            if robot_x is None and pose:
                robot_x = pose["x"]
            if robot_y is None and pose:
                robot_y = pose["y"]
        builder.ingest_points(
            pts,
            robot_x=robot_x,
            robot_y=robot_y,
            count_scan=False,
        )

    def _ingest_topic(self, label: str, topic: str, msg: dict, recv_t: float) -> None:
        if label in LIDAR_NPZ_LABELS:
            self._ingest_lidar(label, topic, msg, recv_t)
            return

        data = msg.get("data")
        row = {
            "seq": len(self._topic_entries(label)),
            "recv_t": recv_t,
            "robot_t": robot_stamp_sec(data if isinstance(data, dict) else {}),
            "topic": topic,
            "type": msg.get("type"),
            "data": strip_heavy_fields(data),
        }
        entry = IndexEntry(recv_t, row)
        self._topic_entries(label).append(entry)

    def _topic_entries(self, label: str) -> list[IndexEntry]:
        return {
            "ROBOTODOM": self.odom,
            "LF_SPORT_MOD_STATE": self.sport,
            "LOW_STATE": self.battery,
            "ULIDAR_STATE": self.ulidar_state,
            "UWB_STATE": self.uwb,
            "MULTIPLE_STATE": self.multiple_state,
            "AUDIO_HUB_PLAY_STATE": self.audio_hub,
        }.get(label, [])

    def _ingest_lidar(self, label: str, topic: str, msg: dict, recv_t: float) -> None:
        data = msg.get("data", {})
        if not isinstance(data, dict):
            return
        inner = data.get("data")
        if not isinstance(inner, dict):
            return
        points = extract_lidar_points(inner)
        if points is None or len(points) == 0:
            return

        origin = np.asarray(data.get("origin", [0, 0, 0]), dtype=np.float64)
        resolution = float(data.get("resolution", 0.05))
        robot_t = data.get("stamp")
        robot_t = float(robot_t) if isinstance(robot_t, (int, float)) else robot_stamp_sec(data)

        seq = len(self.lidar)
        row = {
            "seq": seq,
            "label": label,
            "topic": topic,
            "file": f"{seq:06d}.npz",
            "recv_t": recv_t,
            "robot_t": robot_t,
            "frame_id": data.get("frame_id", "odom"),
            "origin": origin.tolist(),
            "resolution": resolution,
            "point_count": int(len(points)),
        }
        self._lidar_points[seq] = points.astype(np.float32)
        self.lidar.append(IndexEntry(recv_t, row))

    def _ingest_video(self, jpg: bytes, recv_t: float) -> None:
        seq = len(self.video)
        row = {"seq": seq, "file": f"frame_{seq:06d}.jpg", "recv_t": recv_t}
        self._latest_video_jpg = jpg
        self.video.append(IndexEntry(recv_t, row))

    def _reset_buffers(self) -> None:
        self.video.clear()
        self.lidar.clear()
        self.odom.clear()
        self.sport.clear()
        self.battery.clear()
        self.ulidar_state.clear()
        self.uwb.clear()
        self.multiple_state.clear()
        self.audio_hub.clear()
        self._lidar_points.clear()
        self._latest_video_jpg = None
        self._floor_builder = None
        self._floor_lidar_i = 0
        self._floor_odom_i = 0
        self._scan_lidar_start = 0
        self._scan_odom_start = 0

    async def _start_connection(self) -> None:
        if self._connect_task and not self._connect_task.done():
            return
        self._stop = asyncio.Event()
        self._connect_task = asyncio.create_task(self._connection_main())

    async def _stop_connection(self) -> None:
        self._stop.set()

        task = self._connect_task
        if task is not None and not task.done():
            try:
                await asyncio.wait_for(task, timeout=12.0)
            except asyncio.TimeoutError:
                task.cancel()
                try:
                    await asyncio.wait_for(task, timeout=3.0)
                except (asyncio.TimeoutError, asyncio.CancelledError, Exception):
                    pass

        with self._lock:
            conn = self._conn
            self._connect_task = None

        if conn is not None:
            try:
                await asyncio.wait_for(conn.disconnect(), timeout=5.0)
            except Exception:
                pass

        recorder = None
        session_id = None
        with self._lock:
            recorder = self._recorder
            session_id = self._session_id
            self._recorder = None
            self._session_id = None
            self._conn = None
            self.state = "idle"
            self.error = None
            self._start_t = 0.0
            self._last_client_at = 0.0
            self._connecting_since = 0.0

        if recorder is not None:
            try:
                with recorder.lock:
                    recorder._accepting = False
                with self._lock:
                    ip = self._robot_ip
                elapsed = max(0.0, time.time() - recorder.counts.start_t)
                recorder.close({
                    "robot_ip": ip,
                    "session_id": session_id,
                    "duration_s": round(elapsed, 2),
                    "note": _read_note(recorder.root),
                    "tags": read_session_tags(recorder.root),
                    "interrupted": True,
                    "video_fps": 5.0,
                })
            except Exception:
                try:
                    repair_session(recorder.root)
                except Exception:
                    pass

    async def _connection_main(self) -> None:
        with self._lock:
            ip = self._robot_ip
        aes_key = configured_aes_128_key()
        conn_kw: dict[str, Any] = {"connectionMethod": WebRTCConnectionMethod.LocalSTA, "ip": ip}
        if aes_key:
            conn_kw["aes_128_key"] = aes_key
        conn = UnitreeWebRTCConnection(**conn_kw)
        video_task = None
        rpc_task = None
        photo_task = None
        teleop_task = None

        try:
            with self._lock:
                self._reset_buffers()
                self._rpc.clear()
            await conn.connect()

            with self._lock:
                self._conn = conn
                self._start_t = time.time()
                self._last_client_at = time.time()
                self._clear_link_unhealthy()
                self._disconnect_reason = None
                self.state = "connected"
                self.error = None

            self.load_latest_scan_session()

            self._zero_drive_target()
            self._teleop_was_moving = False
            teleop_task = asyncio.create_task(self._teleop_loop())

            await conn.datachannel.disableTrafficSaving(True)
            conn.datachannel.set_decoder(decoder_type="native")
            conn.datachannel.pub_sub.publish_without_callback(RTC_TOPIC["ULIDAR_SWITCH"], "on")
            await asyncio.sleep(0.5)

            subscribe_labels = [
                (label, topic)
                for label, topic in RTC_TOPIC.items()
                if label not in SKIP_SUBSCRIBE
            ]
            priority = ("ULIDAR_ARRAY", "ULIDAR", "ROBOTODOM", "ULIDAR_STATE", "LOW_STATE")
            subscribe_labels.sort(
                key=lambda x: (priority.index(x[0]) if x[0] in priority else 99, x[0])
            )
            core = ("ULIDAR_ARRAY", "ROBOTODOM", "LOW_STATE", "ULIDAR_STATE")
            core_set = set(core)

            def make_cb(label: str, topic: str):
                def cb(msg: dict) -> None:
                    recv_t = time.time()
                    with self._lock:
                        self._ingest_topic(label, topic, msg, recv_t)
                        recorder = self._recorder
                    if recorder is not None:
                        recorder.on_topic(label, topic, msg)
                return cb

            for label, topic in subscribe_labels:
                if label in core_set:
                    conn.datachannel.pub_sub.subscribe(topic, make_cb(label, topic))

            deadline = time.time() + 20
            while time.time() < deadline and not self._stop.is_set():
                with self._lock:
                    has_lidar = len(self.lidar) > 0
                if has_lidar:
                    break
                await asyncio.sleep(0.1)

            for label, topic in subscribe_labels:
                if label not in core_set:
                    conn.datachannel.pub_sub.subscribe(topic, make_cb(label, topic))

            async def _video_loop_local() -> None:
                import cv2

                last_save = 0.0
                min_interval = 0.2

                async def on_track(track):
                    nonlocal last_save
                    while not self._stop.is_set():
                        try:
                            frame = await asyncio.wait_for(track.recv(), timeout=2.0)
                            now = time.time()
                            if now - last_save >= min_interval:
                                img = frame.to_ndarray(format="bgr24")
                                ok, buf = cv2.imencode(".jpg", img)
                                if ok:
                                    jpg = buf.tobytes()
                                    with self._lock:
                                        self._ingest_video(jpg, now)
                                        recorder = self._recorder
                                    if recorder is not None:
                                        recorder.save_video_frame(img, now)
                                last_save = now
                        except asyncio.TimeoutError:
                            continue
                        except Exception:
                            break

                conn.video.add_track_callback(on_track)
                conn.video.switchVideoChannel(True)
                try:
                    while not self._stop.is_set():
                        await asyncio.sleep(0.2)
                finally:
                    conn.video.switchVideoChannel(False)

            video_task = asyncio.create_task(_video_loop_local())

            async def _run_rpc() -> None:
                recorder = _MemoryRpcRecorder(self)
                await run_rpc_probes(conn, recorder, 8.0)
                with self._lock:
                    for name, summary in recorder.snapshots.items():
                        if summary:
                            self._rpc[name] = summary

            rpc_task = asyncio.create_task(_run_rpc())

            disconnected_since: float | None = None
            peer_down_since: float | None = None
            while not self._stop.is_set():
                pc = conn.pc
                pc_state = pc.connectionState if pc is not None else None
                if pc_state in ("failed", "closed"):
                    with self._lock:
                        self._disconnect_reason = f"webrtc_{pc_state}"
                    break
                if not conn.isConnected:
                    if peer_down_since is None:
                        peer_down_since = time.time()
                    elif time.time() - peer_down_since >= PEER_DOWN_GRACE_S:
                        with self._lock:
                            self._disconnect_reason = "peer_closed"
                        break
                else:
                    peer_down_since = None
                if pc_state == "disconnected":
                    if disconnected_since is None:
                        disconnected_since = time.time()
                    elif time.time() - disconnected_since > 20.0:
                        with self._lock:
                            self._disconnect_reason = "webrtc_disconnected"
                        break
                else:
                    disconnected_since = None
                with self._lock:
                    last_client = self._last_client_at
                with self._nav_lock:
                    nav_active = self._nav_state.active
                if (
                    not nav_active
                    and last_client > 0
                    and (time.time() - last_client) > CLIENT_IDLE_TIMEOUT_S
                ):
                    with self._lock:
                        self._disconnect_reason = "idle_timeout"
                    self._stop.set()
                    break
                await asyncio.sleep(0.25)

        except Exception as exc:
            with self._lock:
                self.state = "error"
                self.error = str(exc)
        finally:
            self._stop.set()
            for task in (video_task, rpc_task, photo_task, teleop_task):
                if task is not None and not task.done():
                    task.cancel()
            for task in (video_task, rpc_task, photo_task, teleop_task):
                if task is not None:
                    try:
                        await asyncio.wait_for(task, timeout=2.0)
                    except (asyncio.TimeoutError, asyncio.CancelledError, Exception):
                        pass
            try:
                conn.video.switchVideoChannel(False)
            except Exception:
                pass
            if conn.isConnected:
                try:
                    await asyncio.wait_for(conn.disconnect(), timeout=5.0)
                except Exception:
                    pass
            with self._lock:
                if self._recorder is not None and not self._stop.is_set():
                    try:
                        repair_session(self._recorder.root)
                    except Exception:
                        pass
                if self._stop.is_set():
                    self.state = "idle"
                elif self.state == "connected":
                    self.state = "error"
                    self.error = self.error or "Connection closed"
                self._conn = None


@dataclass
class _MemoryRpcRecorder:
    """Minimal RPC sink for live probes before recording starts."""

    manager: LiveManager
    snapshots: dict[str, Any] = field(default_factory=dict)

    def save_rpc(self, name: str, response: dict) -> None:
        summary = summarize_rpc(name, response)
        if summary:
            self.snapshots[name] = summary
        with self.manager._lock:
            if summary:
                self.manager._rpc[name] = summary
        recorder = self.manager._recorder
        if recorder is not None:
            recorder.save_rpc(name, response)


def _read_note(root: Path) -> str:
    sp = root / "session.json"
    if not sp.exists():
        return ""
    with open(sp, encoding="utf-8") as f:
        return json.load(f).get("note", "")


def _empty_floor_plan(*, upto_t: float) -> dict[str, Any]:
    return {
        "width": 1,
        "height": 1,
        "origin_x": 0.0,
        "origin_y": 0.0,
        "resolution": 0.05,
        "scan_count": 0,
        "threshold": 0.0,
        "zone_count": 0,
        "map_rotation": 0.0,
        "zones_b64": "",
        "walls_b64": "",
        "path": [],
        "upto_t": upto_t,
    }


live_manager = LiveManager()
