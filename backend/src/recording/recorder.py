"""Go2 WebRTC session recording library."""

from __future__ import annotations

import asyncio
import json
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from unitree_webrtc_connect.constants import (
    OBSTACLES_AVOID_API,
    RTC_TOPIC,
    SPORT_CMD,
)

from recording.util import (
    LIDAR_NPZ_LABELS,
    SKIP_SUBSCRIBE,
    extract_lidar_points,
    robot_stamp_sec,
    strip_heavy_fields,
)

DEFAULT_OUTPUT = "recordings"
SCHEMA_VERSION = 1

VUI_GET_SWITCH = 1002
VUI_GET_VOLUME = 1004
VUI_GET_BRIGHTNESS = 1006
MOTION_CHECK_MODE = 1001
AUDIO_GET_LIST = 1001

RPC_PROBES = [
    ("sport_GetState", RTC_TOPIC["SPORT_MOD"], SPORT_CMD["GetState"], None),
    ("sport_GetBodyHeight", RTC_TOPIC["SPORT_MOD"], SPORT_CMD["GetBodyHeight"], None),
    ("sport_GetFootRaiseHeight", RTC_TOPIC["SPORT_MOD"], SPORT_CMD["GetFootRaiseHeight"], None),
    ("sport_GetSpeedLevel", RTC_TOPIC["SPORT_MOD"], SPORT_CMD["GetSpeedLevel"], None),
    ("obstacles_avoid_SwitchGet", RTC_TOPIC["OBSTACLES_AVOID"], OBSTACLES_AVOID_API["SWITCH_GET"], {}),
    ("motion_switcher_CheckMode", RTC_TOPIC["MOTION_SWITCHER"], MOTION_CHECK_MODE, {}),
    ("vui_GetSwitch", RTC_TOPIC["VUI"], VUI_GET_SWITCH, {}),
    ("vui_GetVolume", RTC_TOPIC["VUI"], VUI_GET_VOLUME, {}),
    ("vui_GetBrightness", RTC_TOPIC["VUI"], VUI_GET_BRIGHTNESS, {}),
    ("audiohub_GetAudioList", RTC_TOPIC["AUDIO_HUB_REQ"], AUDIO_GET_LIST, {}),
]


def _count_jsonl(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(1 for line in open(path, encoding="utf-8") if line.strip())


@dataclass
class StreamCounts:
    topics: dict[str, int] = field(default_factory=dict)
    lidar: int = 0
    video: int = 0
    lidar_dropped: int = 0
    start_t: float = field(default_factory=time.time)
    last_report_t: float = field(default_factory=time.time)


class FullSessionRecorder:
    def __init__(self, root: Path, note: str = ""):
        self.root = root
        self.topics_dir = root / "topics"
        self.lidar_dir = root / "lidar"
        self.video_dir = root / "video"
        self.rpc_dir = root / "rpc"
        for d in (self.topics_dir, self.lidar_dir, self.video_dir, self.rpc_dir):
            d.mkdir(parents=True, exist_ok=True)

        self.lock = threading.Lock()
        self.counts = StreamCounts()
        self._topic_files: dict[str, Any] = {}
        self._lidar_index_f = open(self.lidar_dir / "index.jsonl", "w", encoding="utf-8")
        self._video_index_f = open(self.video_dir / "index.jsonl", "w", encoding="utf-8")
        self._lidar_seq = 0
        self._video_seq = 0
        self._topic_seq: dict[str, int] = {}
        self._closed = False
        self._accepting = True

        with open(root / "session.json", "w", encoding="utf-8") as f:
            json.dump({
                "note": note,
                "tags": [],
                "created_at": datetime.now(timezone.utc).isoformat(),
                "schema_version": SCHEMA_VERSION,
                "format": "go2_webrtc_full_v1",
            }, f, indent=2)

    def _topic_handle(self, label: str):
        if label not in self._topic_files:
            self._topic_files[label] = open(
                self.topics_dir / f"{label}.jsonl", "w", encoding="utf-8"
            )
            self._topic_seq[label] = 0
        return self._topic_files[label]

    def _write_jsonl(self, fh, row: dict) -> None:
        fh.write(json.dumps(row, separators=(",", ":")) + "\n")
        fh.flush()

    def on_topic(self, label: str, topic: str, msg: dict) -> None:
        if not self._accepting:
            return
        recv_t = time.time()
        if label in LIDAR_NPZ_LABELS:
            self._save_lidar(label, topic, msg, recv_t)
            return

        data = msg.get("data")
        row = {
            "seq": self._topic_seq.get(label, 0),
            "recv_t": recv_t,
            "robot_t": robot_stamp_sec(data if isinstance(data, dict) else {}),
            "topic": topic,
            "type": msg.get("type"),
            "data": strip_heavy_fields(data),
        }
        with self.lock:
            fh = self._topic_handle(label)
            self._write_jsonl(fh, row)
            self._topic_seq[label] = self._topic_seq.get(label, 0) + 1
            self.counts.topics[label] = self._topic_seq[label]

    def _save_lidar(self, label: str, topic: str, msg: dict, recv_t: float) -> None:
        data = msg.get("data", {})
        if not isinstance(data, dict):
            return
        inner = data.get("data")
        if not isinstance(inner, dict):
            return

        points = extract_lidar_points(inner)
        if points is None or len(points) == 0:
            with self.lock:
                self.counts.lidar_dropped += 1
            return

        origin = np.asarray(data.get("origin", [0, 0, 0]), dtype=np.float64)
        resolution = float(data.get("resolution", 0.05))
        robot_t = data.get("stamp")
        robot_t = float(robot_t) if isinstance(robot_t, (int, float)) else robot_stamp_sec(data)

        seq = self._lidar_seq
        fname = f"{seq:06d}.npz"
        np.savez_compressed(
            self.lidar_dir / fname,
            points=points.astype(np.float32),
            origin=origin,
            resolution=np.float32(resolution),
        )

        row = {
            "seq": seq,
            "label": label,
            "topic": topic,
            "file": fname,
            "recv_t": recv_t,
            "robot_t": robot_t,
            "frame_id": data.get("frame_id", "odom"),
            "origin": origin.tolist(),
            "resolution": resolution,
            "point_count": int(len(points)),
        }
        with self.lock:
            self._write_jsonl(self._lidar_index_f, row)
            self._lidar_seq += 1
            self.counts.lidar += 1

    def save_video_frame(self, bgr: np.ndarray, recv_t: float) -> None:
        seq = self._video_seq
        fname = f"frame_{seq:06d}.jpg"
        cv2.imwrite(str(self.video_dir / fname), bgr)
        row = {"seq": seq, "file": fname, "recv_t": recv_t, "shape": list(bgr.shape)}
        with self.lock:
            self._write_jsonl(self._video_index_f, row)
            self._video_seq += 1
            self.counts.video += 1

    def save_rpc(self, name: str, response: dict) -> None:
        path = self.rpc_dir / f"{name}.json"
        with open(path, "w", encoding="utf-8") as f:
            json.dump(strip_heavy_fields(response), f, indent=2)

    def snapshot(self) -> StreamCounts:
        with self.lock:
            return StreamCounts(
                topics=dict(self.counts.topics),
                lidar=self.counts.lidar,
                video=self.counts.video,
                lidar_dropped=self.counts.lidar_dropped,
                start_t=self.counts.start_t,
                last_report_t=self.counts.last_report_t,
            )

    def close(self, meta: dict[str, Any]) -> Path:
        with self.lock:
            if self._closed:
                return self.root / "manifest.json"
            self._accepting = False
            self._closed = True
            for fh in self._topic_files.values():
                if not fh.closed:
                    fh.close()
            for fh in (self._lidar_index_f, self._video_index_f):
                if not fh.closed:
                    fh.close()

        topic_stats = {}
        for label in sorted(set(list(self._topic_seq.keys()) + list(RTC_TOPIC.keys()))):
            path = self.topics_dir / f"{label}.jsonl"
            n = _count_jsonl(path)
            if n > 0:
                topic_stats[label] = {"topic": RTC_TOPIC.get(label, ""), "count": n}

        manifest = {
            "schema_version": SCHEMA_VERSION,
            "format": "go2_webrtc_full_v1",
            "created_at": datetime.now(timezone.utc).isoformat(),
            "subscribed": [lbl for lbl in RTC_TOPIC if lbl not in SKIP_SUBSCRIBE],
            "skipped_subscribe": sorted(SKIP_SUBSCRIBE),
            "streams": {
                "topics": topic_stats,
                "lidar": {"count": self.counts.lidar, "index": "lidar/index.jsonl"},
                "video": {"count": self.counts.video, "index": "video/index.jsonl"},
                "rpc": sorted(p.name for p in self.rpc_dir.glob("*.json")),
            },
            **meta,
        }
        manifest_path = self.root / "manifest.json"
        with open(manifest_path, "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2)

        latest = self.root.parent / "latest"
        latest.unlink(missing_ok=True)
        latest.symlink_to(self.root.name)
        return manifest_path


def repair_session(path: str | Path) -> Path:
    root = Path(path).resolve()
    session = {}
    sp = root / "session.json"
    if sp.exists():
        with open(sp, encoding="utf-8") as f:
            session = json.load(f)

    topic_stats = {}
    topics_dir = root / "topics"
    if topics_dir.exists():
        for p in sorted(topics_dir.glob("*.jsonl")):
            label = p.stem
            n = _count_jsonl(p)
            if n:
                topic_stats[label] = {"topic": RTC_TOPIC.get(label, ""), "count": n}

    manifest = {
        "schema_version": SCHEMA_VERSION,
        "format": "go2_webrtc_full_v1",
        "repaired_at": datetime.now(timezone.utc).isoformat(),
        "note": session.get("note", ""),
        "tags": session.get("tags", []),
        "session_id": root.name,
        "interrupted": True,
        "streams": {
            "topics": topic_stats,
            "lidar": {"count": _count_jsonl(root / "lidar" / "index.jsonl")},
            "video": {"count": _count_jsonl(root / "video" / "index.jsonl")},
            "rpc": sorted(p.name for p in (root / "rpc").glob("*.json")),
        },
    }
    path_out = root / "manifest.json"
    with open(path_out, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
    latest = root.parent / "latest"
    latest.unlink(missing_ok=True)
    latest.symlink_to(root.name)
    return path_out


async def rpc_fetch(conn, name, topic, api_id, parameter, timeout):
    opts: dict[str, Any] = {"api_id": api_id}
    if parameter is not None:
        opts["parameter"] = parameter if isinstance(parameter, str) else json.dumps(parameter)
    try:
        resp = await asyncio.wait_for(
            conn.datachannel.pub_sub.publish_request_new(topic, opts),
            timeout=timeout,
        )
        return name, resp, None
    except Exception as exc:
        return name, None, str(exc)


async def capture_photo(conn, recorder, timeout):
    try:
        resp = await asyncio.wait_for(
            conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["FRONT_PHOTO_REQ"], {"api_id": 1001},
            ),
            timeout=timeout,
        )
        recorder.save_rpc("front_photo_response", resp)
        data = resp.get("data", {})
        if isinstance(data, dict):
            raw = data.get("data")
            if isinstance(raw, (bytes, bytearray)):
                (recorder.root / "video" / "front_photo.jpg").write_bytes(bytes(raw))
                return True
    except Exception:
        pass
    return False


async def run_rpc_probes(conn, recorder, timeout: float) -> None:
    for name, topic, api_id, param in RPC_PROBES:
        _, resp, err = await rpc_fetch(conn, name, topic, api_id, param, timeout)
        if resp:
            recorder.save_rpc(name, resp)
        elif err:
            recorder.save_rpc(f"{name}_error", {"error": err})
