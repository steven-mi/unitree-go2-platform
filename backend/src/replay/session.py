"""Replay a single recorded session — frames, lidar, and floor plans."""

from __future__ import annotations

import base64
import threading
from bisect import bisect_left
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np

from config import FLOOR_PLAN_REV, cfg_float, load_config
from domain.models import IndexEntry
from floorplan.builder import FloorPlanBuilder, FloorPlanResult, floor_plan_to_api
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

_DEFAULT_RESOLUTION = cfg_float(load_config(), "floorplan.resolution")


@dataclass
class SessionReplay:
    session_id: str
    root: Path
    manifest: dict[str, Any]
    session_meta: dict[str, Any]
    t0: float
    duration: float
    video: list[IndexEntry] = field(default_factory=list)
    lidar: list[IndexEntry] = field(default_factory=list)
    odom: list[IndexEntry] = field(default_factory=list)
    sport: list[IndexEntry] = field(default_factory=list)
    battery: list[IndexEntry] = field(default_factory=list)
    ulidar_state: list[IndexEntry] = field(default_factory=list)
    uwb: list[IndexEntry] = field(default_factory=list)
    multiple_state: list[IndexEntry] = field(default_factory=list)
    audio_hub: list[IndexEntry] = field(default_factory=list)
    rpc: dict[str, Any] = field(default_factory=dict)
    services: list[dict[str, Any]] = field(default_factory=list)
    _floor_builder: FloorPlanBuilder | None = field(default=None, repr=False)
    _floor_plan_rev: int = field(default=0, repr=False)
    _floor_synced_t: float = field(default=-1.0, repr=False)
    _floor_lidar_i: int = field(default=0, repr=False)
    _floor_odom_i: int = field(default=0, repr=False)
    _floor_lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _odom_times: list[float] | None = field(default=None, repr=False)

    def rel_t(self, recv_t: float) -> float:
        return max(0.0, recv_t - self.t0)

    def _odom_times_list(self) -> list[float]:
        if self._odom_times is None:
            self._odom_times = [self.rel_t(e.recv_t) for e in self.odom]
        return self._odom_times

    def _pose_at_t(self, t: float) -> Optional[dict[str, float]]:
        if not self.odom:
            return None
        times = self._odom_times_list()
        i = bisect_left(times, t)
        if i == 0:
            entry = self.odom[0]
        elif i >= len(self.odom):
            entry = self.odom[-1]
        else:
            before, after = self.odom[i - 1], self.odom[i]
            entry = after if abs(times[i] - t) < abs(t - times[i - 1]) else before
        return parse_pose(entry.payload)

    def _nearest(self, entries: list[IndexEntry], t: float) -> Optional[IndexEntry]:
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

    def frame_at(self, t: float) -> dict[str, Any]:
        t = max(0.0, min(t, self.duration))
        video = self._nearest(self.video, t)
        lidar = self._nearest(self.lidar, t)
        odom = self._nearest(self.odom, t)
        sport = self._nearest(self.sport, t)
        battery = self._nearest(self.battery, t)
        ulidar_state = self._nearest(self.ulidar_state, t)
        uwb = self._nearest(self.uwb, t)
        multiple_state = self._nearest(self.multiple_state, t)
        audio_hub = self._nearest(self.audio_hub, t)

        pose = parse_pose(odom.payload if odom else None)
        if pose is None and sport:
            pose = parse_sport_pose(sport.payload)

        battery_info = parse_battery_state(battery.payload if battery else None)

        out: dict[str, Any] = {
            "t": round(t, 3),
            "duration": round(self.duration, 3),
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
            out["video"] = {
                "seq": video.payload.get("seq"),
                "file": video.payload.get("file"),
                "url": f"/api/recordings/{self.session_id}/video/{video.payload['file']}",
            }
        if lidar:
            out["lidar"] = {
                "seq": lidar.payload.get("seq"),
                "file": lidar.payload.get("file"),
                "point_count": lidar.payload.get("point_count"),
                "url": f"/api/recordings/{self.session_id}/lidar/{lidar.payload['seq']}",
            }
        return out

    def _load_lidar_array(self, seq: int, max_points: int = 8000) -> tuple[np.ndarray, dict[str, Any]]:
        row = next((e.payload for e in self.lidar if e.payload.get("seq") == seq), None)
        if row is None:
            raise FileNotFoundError(f"lidar seq {seq} not found")
        path = self.root / "lidar" / row["file"]
        with np.load(path) as data:
            points = np.asarray(data["points"], dtype=np.float32)
        if max_points > 0 and len(points) > max_points:
            idx = np.linspace(0, len(points) - 1, max_points, dtype=int)
            points = points[idx]
        return points, row

    def load_lidar_points_binary(self, seq: int, max_points: int = 0) -> bytes:
        points, _ = self._load_lidar_array(seq, max_points=max_points)
        return points.tobytes()

    def _floor_plan_data_limit(self, upto_t: float | None) -> float:
        t_limit = self.duration if upto_t is None else max(0.0, min(float(upto_t), self.duration))
        bootstrap_t = 0.0
        if self.lidar:
            bootstrap_t = max(bootstrap_t, self.rel_t(self.lidar[0].recv_t))
        if self.odom:
            bootstrap_t = max(bootstrap_t, self.rel_t(self.odom[0].recv_t))
        return max(t_limit, bootstrap_t) if t_limit < bootstrap_t else t_limit

    def _ensure_floor_builder(self, *, resolution: float) -> FloorPlanBuilder:
        if self._floor_builder is None or self._floor_plan_rev != FLOOR_PLAN_REV:
            self._floor_builder = FloorPlanBuilder(resolution=resolution)
            self._floor_plan_rev = FLOOR_PLAN_REV
            self._floor_synced_t = -1.0
            self._floor_lidar_i = 0
            self._floor_odom_i = 0
        return self._floor_builder

    def _ingest_floor_plan(
        self,
        builder: FloorPlanBuilder,
        data_limit: float,
        scan_stride: int,
        *,
        lidar_i: int,
        odom_i: int,
    ) -> tuple[int, int]:
        stride = max(1, scan_stride)
        odom_stride = max(1, stride * 2)

        while lidar_i < len(self.lidar):
            entry = self.lidar[lidar_i]
            if self.rel_t(entry.recv_t) > data_limit:
                break
            if lidar_i % stride == 0:
                row = entry.payload
                path = self.root / "lidar" / row["file"]
                with np.load(path, mmap_mode="r") as data:
                    pts = np.asarray(data["points"], dtype=np.float32)
                t_scan = self.rel_t(entry.recv_t)
                pose = self._pose_at_t(t_scan)
                builder.ingest_points(
                    pts,
                    robot_x=pose["x"] if pose else None,
                    robot_y=pose["y"] if pose else None,
                )
            lidar_i += 1

        while odom_i < len(self.odom):
            entry = self.odom[odom_i]
            if self.rel_t(entry.recv_t) > data_limit:
                break
            if odom_i % odom_stride == 0:
                pose = parse_pose(entry.payload)
                if pose is not None:
                    builder.append_path(pose["x"], pose["y"])
            odom_i += 1

        return lidar_i, odom_i

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
        path = self.root / "lidar" / row["file"]
        with np.load(path, mmap_mode="r") as data:
            pts = np.asarray(data["points"], dtype=np.float32)
        if entry is not None:
            t_scan = self.rel_t(entry.recv_t)
            pose = self._pose_at_t(t_scan)
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

    def build_floor_plan_result(
        self,
        upto_t: float | None = None,
        *,
        robot_x: float | None = None,
        robot_y: float | None = None,
        resolution: float = _DEFAULT_RESOLUTION,
        scan_stride: int = 1,
        crop: bool = True,
        lidar_seq: int | None = None,
    ) -> FloorPlanResult | None:
        t_limit = self.duration if upto_t is None else max(0.0, min(float(upto_t), self.duration))
        data_limit = self._floor_plan_data_limit(upto_t)

        builder = self._ensure_floor_builder(resolution=resolution)

        if t_limit < self._floor_synced_t - 0.05:
            builder.reset()
            self._floor_synced_t = -1.0
            self._floor_lidar_i = 0
            self._floor_odom_i = 0

        self._floor_lidar_i, self._floor_odom_i = self._ingest_floor_plan(
            builder,
            data_limit,
            scan_stride,
            lidar_i=self._floor_lidar_i,
            odom_i=self._floor_odom_i,
        )
        if lidar_seq is not None:
            self._ingest_lidar_seq(
                builder,
                lidar_seq,
                robot_x=robot_x,
                robot_y=robot_y,
            )
        self._floor_synced_t = t_limit
        if builder.scan_count == 0 and not builder._has_wall_data():
            return None
        return builder.build(crop=crop, robot_x=robot_x, robot_y=robot_y)

    def build_floor_plan(
        self,
        upto_t: float | None = None,
        *,
        robot_x: float | None = None,
        robot_y: float | None = None,
        resolution: float = _DEFAULT_RESOLUTION,
        scan_stride: int = 1,
        crop: bool = True,
        lidar_seq: int | None = None,
    ) -> dict[str, Any]:
        with self._floor_lock:
            return self._build_floor_plan_unlocked(
                upto_t=upto_t,
                robot_x=robot_x,
                robot_y=robot_y,
                resolution=resolution,
                scan_stride=scan_stride,
                crop=crop,
                lidar_seq=lidar_seq,
            )

    def _build_floor_plan_unlocked(
        self,
        upto_t: float | None = None,
        *,
        robot_x: float | None = None,
        robot_y: float | None = None,
        resolution: float = _DEFAULT_RESOLUTION,
        scan_stride: int = 1,
        crop: bool = True,
        lidar_seq: int | None = None,
    ) -> dict[str, Any]:
        t_limit = self.duration if upto_t is None else max(0.0, min(float(upto_t), self.duration))
        plan = self.build_floor_plan_result(
            upto_t=upto_t,
            robot_x=robot_x,
            robot_y=robot_y,
            resolution=resolution,
            scan_stride=scan_stride,
            crop=crop,
            lidar_seq=lidar_seq,
        )
        if plan is None:
            return empty_floor_plan_api(upto_t=t_limit)
        return floor_plan_to_api(plan, upto_t=t_limit)


def empty_floor_plan_api(*, upto_t: float) -> dict[str, Any]:
    zones = np.zeros((1, 1), dtype=np.uint8)
    walls = np.zeros((1, 1), dtype=np.uint8)
    return {
        "width": 1,
        "height": 1,
        "origin_x": 0.0,
        "origin_y": 0.0,
        "resolution": 0.05,
        "scan_count": 0,
        "threshold": 0,
        "zone_count": 0,
        "map_rotation": 0.0,
        "zones_b64": base64.b64encode(zones.tobytes()).decode("ascii"),
        "walls_b64": base64.b64encode(walls.tobytes()).decode("ascii"),
        "path": [],
        "upto_t": round(upto_t, 3),
    }
