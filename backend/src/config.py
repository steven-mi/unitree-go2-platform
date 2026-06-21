"""Runtime configuration.

`config.yml` (repo root) is the single source of truth for every tunable
parameter. There are no hardcoded parameter defaults in the codebase: each value
is read straight from `config.yml`, and a missing key raises `ConfigError` rather
than silently falling back.

Values may reference environment variables with shell-style placeholders, which
are expanded when the file is loaded:

    robot_ip: ${ROBOT_IP}                 # required env var (empty if unset)
    robot_ip: ${ROBOT_IP:-0.0.0.0}   # with a fallback when unset

Only deployment *paths* (config/recordings/scans locations) come straight from
the environment, since they determine where `config.yml` itself is found.
"""

from __future__ import annotations

import ipaddress
import os
import re
from pathlib import Path
from typing import Any

import yaml

_PKG_ROOT = Path(__file__).resolve().parent
_BACKEND_ROOT = _PKG_ROOT.parent

# Matches ${VAR} and ${VAR:-default} (default may be empty or contain anything
# except a closing brace).
_ENV_PLACEHOLDER = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)(?::-([^}]*))?\}")


class ConfigError(RuntimeError):
    """Raised when config.yml is missing, malformed, or missing a required key."""


def project_root() -> Path:
    if raw := os.environ.get("PROJECT_ROOT"):
        return Path(raw)
    return _BACKEND_ROOT.parent


def config_path() -> Path:
    if raw := os.environ.get("CONFIG_PATH"):
        return Path(raw)
    return project_root() / "config.yml"


def _expand_env(value: Any) -> Any:
    """Recursively expand ${VAR} / ${VAR:-default} placeholders in string values."""
    if isinstance(value, str):
        def repl(m: re.Match[str]) -> str:
            name, default = m.group(1), m.group(2)
            return os.environ.get(name, default if default is not None else "")

        return _ENV_PLACEHOLDER.sub(repl, value)
    if isinstance(value, dict):
        return {k: _expand_env(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_expand_env(v) for v in value]
    return value


def load_config(*, expand: bool = True) -> dict[str, Any]:
    """Read and parse config.yml. Raises ConfigError if missing or not a mapping.

    With ``expand`` (default), ``${VAR}`` placeholders in string values are
    replaced with environment variables. Pass ``expand=False`` to get the raw
    file contents (used when rewriting the file so placeholders are preserved).
    """
    path = config_path()
    if not path.is_file():
        raise ConfigError(f"Config file not found: {path}")
    with path.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ConfigError(f"Config file must be a mapping: {path}")
    return _expand_env(data) if expand else data


def save_config(data: dict[str, Any]) -> None:
    path = config_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, default_flow_style=False, sort_keys=False)


# --- Typed accessors -------------------------------------------------------
# Navigate a dotted path (e.g. "obstacle.stop_m") within a loaded config
# dict and coerce to the requested type. A missing key is a hard error so that
# config.yml stays the single, complete source of truth.


def _navigate(cfg: dict[str, Any], dotted: str) -> Any:
    node: Any = cfg
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            raise ConfigError(f"Missing required config key: '{dotted}' in {config_path()}")
        node = node[part]
    return node


def cfg_float(cfg: dict[str, Any], dotted: str) -> float:
    return float(_navigate(cfg, dotted))


def cfg_int(cfg: dict[str, Any], dotted: str) -> int:
    return int(_navigate(cfg, dotted))


def cfg_bool(cfg: dict[str, Any], dotted: str) -> bool:
    value = _navigate(cfg, dotted)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def cfg_str(cfg: dict[str, Any], dotted: str) -> str:
    return str(_navigate(cfg, dotted))


# --- High-level getters (read fresh so config.yml edits take effect live) ---


def configured_robot_ip() -> str:
    ip = cfg_str(load_config(), "robot_ip").strip()
    if not ip:
        raise ConfigError("'robot_ip' resolved to empty (set it in config.yml or via $ROBOT_IP)")
    return ip


def configured_aes_128_key() -> str | None:
    cleaned = cfg_str(load_config(), "aes_128_key").strip()
    return cleaned or None


def configured_nav_speed() -> float:
    """Point-and-go cruise speed, clamped to nav.min_speed..nav.max_speed."""
    cfg = load_config()
    speed = cfg_float(cfg, "nav.speed")
    lo = cfg_float(cfg, "nav.min_speed")
    hi = cfg_float(cfg, "nav.max_speed")
    return max(lo, min(hi, speed))


def configured_planner_clearance() -> dict[str, float]:
    """A* clearance knobs from config.yml (`planner:`), read fresh per plan."""
    cfg = load_config()
    return {
        "robot_radius_m": cfg_float(cfg, "planner.robot_radius_m"),
        "centering_weight": cfg_float(cfg, "planner.centering_weight"),
        "centering_clearance_m": cfg_float(cfg, "planner.centering_clearance_m"),
        "corner_smoothing_iters": cfg_int(cfg, "planner.corner_smoothing_iters"),
        "corner_smoothing_ratio": cfg_float(cfg, "planner.corner_smoothing_ratio"),
    }


# Floor-plan cache revision; bump in config.yml to force regeneration.
FLOOR_PLAN_REV: int = cfg_int(load_config(), "floorplan.revision")


# --- Validation / dashboard Settings ---------------------------------------


def validate_robot_ip(ip: str) -> str:
    cleaned = ip.strip()
    if not cleaned:
        raise ValueError("Robot IP address is required")
    try:
        ipaddress.ip_address(cleaned)
    except ValueError as exc:
        raise ValueError("Enter a valid IPv4 or IPv6 address") from exc
    return cleaned


def validate_aes_128_key(key: str | None) -> str | None:
    if key is None:
        return None
    cleaned = key.strip()
    if not cleaned:
        return None
    try:
        raw = bytes.fromhex(cleaned.lower())
    except ValueError as exc:
        raise ValueError("AES key must be 32 hex characters") from exc
    if len(raw) != 16:
        raise ValueError("AES key must be 32 hex characters (16 bytes)")
    return cleaned.lower()


def set_app_settings(*, robot_ip: str, aes_128_key: str | None = None) -> tuple[str, str | None]:
    ip = validate_robot_ip(robot_ip)
    key = validate_aes_128_key(aes_128_key)
    # Load raw (unexpanded) so ${VAR} placeholders in other keys are preserved.
    cfg = load_config(expand=False)
    cfg["robot_ip"] = ip
    cfg["aes_128_key"] = key or ""
    save_config(cfg)
    return ip, key


# --- Deployment paths (env-overridable) ------------------------------------


def recordings_root() -> Path:
    if raw := os.environ.get("RECORDINGS_DIR"):
        return Path(raw)
    # Docker mounts recordings at /app/recordings; local dev uses ../recordings.
    nested = _BACKEND_ROOT / "recordings"
    if nested.is_dir():
        return nested
    return _BACKEND_ROOT.parent / "recordings"


def scans_root() -> Path:
    if raw := os.environ.get("SCANS_DIR"):
        return Path(raw)
    return _BACKEND_ROOT.parent / "scans"
