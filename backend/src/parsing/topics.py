"""Parse robot topic payloads into dashboard-friendly structures."""

from __future__ import annotations

import json
import math
from typing import Any, Optional


def row_data(row: Optional[dict[str, Any]]) -> Any:
    if not row:
        return None
    data = row.get("data")
    if isinstance(data, str):
        try:
            return json.loads(data)
        except json.JSONDecodeError:
            return None
    if isinstance(data, dict):
        inner = data.get("data")
        if isinstance(inner, str):
            try:
                return json.loads(inner)
            except json.JSONDecodeError:
                return None
        if isinstance(inner, dict):
            return inner
        return data
    return None


def topic_payload(row: Optional[dict[str, Any]]) -> Optional[dict[str, Any]]:
    data = row_data(row)
    return data if isinstance(data, dict) else None


def vec3(value: Any) -> Optional[list[float]]:
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return [float(value[0]), float(value[1]), float(value[2])]
    return None


def parse_pose(row: Optional[dict[str, Any]]) -> Optional[dict[str, Any]]:
    if not row:
        return None
    data = row.get("data", {})
    if not isinstance(data, dict):
        return None
    pose = data.get("pose")
    if not isinstance(pose, dict):
        return None
    pos = pose.get("position", {})
    ori = pose.get("orientation", {})
    if not isinstance(pos, dict):
        return None
    x, y, z = float(pos.get("x", 0)), float(pos.get("y", 0)), float(pos.get("z", 0))
    qx = float(ori.get("x", 0))
    qy = float(ori.get("y", 0))
    qz = float(ori.get("z", 0))
    qw = float(ori.get("w", 1))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return {
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "qx": qx,
        "qy": qy,
        "qz": qz,
        "qw": qw,
    }


def parse_sport_pose(row: Optional[dict[str, Any]]) -> Optional[dict[str, Any]]:
    if not row:
        return None
    data = row.get("data", {})
    if not isinstance(data, dict) or "position" not in data:
        return None
    pos = data["position"]
    imu = data.get("imu_state", {})
    yaw = None
    if isinstance(imu, dict) and "rpy" in imu:
        rpy = imu["rpy"]
        if isinstance(rpy, (list, tuple)) and len(rpy) >= 3:
            yaw = float(rpy[2])
    return {
        "x": float(pos[0]),
        "y": float(pos[1]),
        "z": float(pos[2]),
        "yaw": yaw,
    }


def parse_velocity(row: Optional[dict[str, Any]]) -> Optional[list[float]]:
    if not row:
        return None
    data = row.get("data", {})
    if not isinstance(data, dict):
        return None
    vel = data.get("velocity")
    if isinstance(vel, (list, tuple)) and len(vel) >= 2:
        return [float(vel[0]), float(vel[1]), float(vel[2]) if len(vel) > 2 else 0.0]
    return None


def parse_battery(row: Optional[dict[str, Any]]) -> Optional[float]:
    info = parse_battery_state(row)
    if not info:
        return None
    v = info.get("voltage")
    return float(v) if isinstance(v, (int, float)) else None


def parse_sport(row: Optional[dict[str, Any]]) -> Optional[dict[str, Any]]:
    data = topic_payload(row)
    if not data:
        return None

    imu = data.get("imu_state", {})
    if not isinstance(imu, dict):
        imu = {}

    rpy = vec3(imu.get("rpy"))
    gyro = vec3(imu.get("gyroscope"))
    accel = vec3(imu.get("accelerometer"))
    temp = imu.get("temperature")

    obstacle = data.get("range_obstacle")
    range_obstacle = None
    if isinstance(obstacle, (list, tuple)) and len(obstacle) >= 4:
        range_obstacle = [float(obstacle[i]) for i in range(4)]

    return {
        "mode": int(data.get("mode", 0)),
        "gait_type": int(data.get("gait_type", 0)),
        "body_height": float(data.get("body_height", 0)),
        "yaw_rate": float(data.get("yaw_speed", 0)),
        "error_code": int(data.get("error_code", 0)),
        "range_obstacle": range_obstacle,
        "imu": {
            "rpy": rpy,
            "gyro": gyro,
            "accel": accel,
            "temperature": int(temp) if isinstance(temp, (int, float)) else None,
        },
    }


def parse_battery_state(row: Optional[dict[str, Any]]) -> Optional[dict[str, Any]]:
    data = topic_payload(row)
    if not data:
        return None

    bms = data.get("bms_state", {})
    if not isinstance(bms, dict):
        bms = {}

    foot = data.get("foot_force")
    foot_force = None
    if isinstance(foot, (list, tuple)) and len(foot) >= 4:
        foot_force = [float(foot[i]) for i in range(4)]

    voltage = data.get("power_v")
    soc = bms.get("soc")
    current = bms.get("current")
    temp = data.get("temperature_ntc1")

    motors = []
    for motor in data.get("motor_state", []):
        if not isinstance(motor, dict):
            continue
        q = motor.get("q")
        mt = motor.get("temperature")
        motors.append({
            "angle": float(q) if isinstance(q, (int, float)) else 0.0,
            "temperature": int(mt) if isinstance(mt, (int, float)) else None,
        })

    return {
        "voltage": float(voltage) if isinstance(voltage, (int, float)) else None,
        "soc": int(soc) if isinstance(soc, (int, float)) else None,
        "current_ma": int(current) if isinstance(current, (int, float)) else None,
        "temperature_c": float(temp) if isinstance(temp, (int, float)) else None,
        "foot_force": foot_force,
        "motors": motors or None,
    }


def parse_lidar_state(row: Optional[dict[str, Any]]) -> Optional[dict[str, Any]]:
    data = topic_payload(row)
    if not data:
        return None

    cloud_freq = data.get("cloud_frequency")
    error_state = data.get("error_state")
    dirty = data.get("dirty_percentage")
    cloud_size = data.get("cloud_size")

    return {
        "cloud_frequency": float(cloud_freq) if isinstance(cloud_freq, (int, float)) else None,
        "error_state": int(error_state) if isinstance(error_state, (int, float)) else None,
        "dirty_percentage": float(dirty) if isinstance(dirty, (int, float)) else None,
        "cloud_size": int(cloud_size) if isinstance(cloud_size, (int, float)) else None,
    }


def parse_uwb(row: Optional[dict[str, Any]]) -> Optional[dict[str, Any]]:
    data = topic_payload(row)
    if not data:
        return None

    joystick = data.get("joystick")
    joy = None
    if isinstance(joystick, (list, tuple)) and len(joystick) >= 2:
        joy = [float(joystick[0]), float(joystick[1])]

    def f(key: str) -> Optional[float]:
        v = data.get(key)
        return float(v) if isinstance(v, (int, float)) else None

    return {
        "distance": f("distance_est"),
        "yaw": f("yaw_est"),
        "pitch": f("pitch_est"),
        "orientation": f("orientation_est"),
        "joystick": joy,
        "buttons": int(data["buttons"]) if isinstance(data.get("buttons"), (int, float)) else None,
        "joy_mode": int(data["joy_mode"]) if isinstance(data.get("joy_mode"), (int, float)) else None,
        "enabled_from_app": bool(data.get("enabled_from_app")),
    }


def parse_system(row: Optional[dict[str, Any]]) -> Optional[dict[str, Any]]:
    data = row_data(row)
    if not isinstance(data, dict):
        return None
    return {
        "volume": int(data["volume"]) if isinstance(data.get("volume"), (int, float)) else None,
        "brightness": int(data["brightness"]) if isinstance(data.get("brightness"), (int, float)) else None,
        "obstacles_avoid": bool(data["obstaclesAvoidSwitch"]) if "obstaclesAvoidSwitch" in data else None,
        "uwb_switch": bool(data["uwbSwitch"]) if "uwbSwitch" in data else None,
    }


def parse_audio(row: Optional[dict[str, Any]]) -> Optional[dict[str, Any]]:
    data = row_data(row)
    if not isinstance(data, dict):
        return None
    name = data.get("current_audio_custom_name") or data.get("current_audio_unique_id") or ""
    return {
        "play_state": str(data.get("play_state", "")),
        "is_playing": bool(data.get("is_playing")),
        "track": str(name) if name else None,
    }
