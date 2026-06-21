"""
Shared helpers for Go2 WebRTC session recording.
"""

from __future__ import annotations

import base64
import json
from typing import Any, Optional

import numpy as np

# Topics that are RPC/publish-only — subscribe to the rest.
SKIP_SUBSCRIBE = frozenset({
    "ULIDAR_SWITCH",
    "FRONT_PHOTO_REQ",
    "SPORT_MOD",
    "VUI",
    "OBSTACLES_AVOID",
    "MOTION_SWITCHER",
    "AUDIO_HUB_REQ",
    "BASH_REQ",
    "UWB_REQ",
    "GAS_SENSOR_REQ",
    "ASSISTANT_RECORDER",
    "LIDAR_MAPPING_CMD",
    "PROGRAMMING_ACTUATOR_CMD",
})

LIDAR_NPZ_LABELS = frozenset({"ULIDAR_ARRAY"})


def extract_lidar_points(inner: dict) -> Optional[np.ndarray]:
    if not isinstance(inner, dict):
        return None
    if "points" in inner:
        pts = np.asarray(inner["points"], dtype=np.float64).reshape(-1, 3)
        return pts[np.isfinite(pts).all(axis=1)]
    if "positions" not in inner:
        return None
    raw = np.asarray(inner["positions"])
    if raw.size == 0:
        return None
    fc = int(inner.get("face_count", 0) or 0)
    n = min(fc * 12 if fc > 0 else raw.size, raw.size)
    if raw.dtype == np.uint8:
        pts = np.frombuffer(raw[:n].tobytes(), dtype=np.float32).reshape(-1, 3)
    else:
        pts = raw.astype(np.float32).reshape(-1, 3)
    good = np.isfinite(pts).all(axis=1) & (np.abs(pts) < 100).all(axis=1)
    return pts[good]


def strip_heavy_fields(obj: Any) -> Any:
    if isinstance(obj, np.ndarray):
        if obj.size > 64:
            return {"__omitted__": "ndarray", "shape": list(obj.shape), "dtype": str(obj.dtype)}
        return obj.tolist()
    if isinstance(obj, (bytes, bytearray)):
        if len(obj) > 256:
            return {"__omitted__": "bytes", "len": len(obj)}
        return {"__b64__": base64.b64encode(bytes(obj)).decode("ascii")}
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            if k in ("points", "positions", "uvs", "indices") and hasattr(v, "__len__") and len(v) > 64:
                out[f"{k}_omitted"] = len(v)
            else:
                out[k] = strip_heavy_fields(v)
        return out
    if isinstance(obj, list):
        return [strip_heavy_fields(v) for v in obj]
    if isinstance(obj, (int, float, str, bool)) or obj is None:
        return obj
    return str(obj)


def robot_stamp_sec(data: dict) -> Optional[float]:
    if not isinstance(data, dict):
        return None
    header = data.get("header")
    if isinstance(header, dict):
        stamp = header.get("stamp")
        if isinstance(stamp, dict):
            return float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) * 1e-9
    stamp = data.get("stamp")
    if isinstance(stamp, (int, float)):
        return float(stamp)
    return None
