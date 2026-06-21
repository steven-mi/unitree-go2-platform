"""Persist saved floor plans, raw lidar scans, and routes for scan sessions."""

from __future__ import annotations

import json
import shutil
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from config import scans_root

LATEST_SCAN_ID = "latest"

# Compact occupancy-grid snapshot used to resume mapping (replaces storing every
# raw lidar frame under the scan).
GRID_FILENAME = "floor_grid.npz"


def grid_path(scan_id: str) -> Path:
    return _scan_dir(scan_id) / GRID_FILENAME


def ensure_scans_root() -> Path:
    root = scans_root()
    root.mkdir(parents=True, exist_ok=True)
    return root


def _scan_dir(scan_id: str) -> Path:
    root = scans_root().resolve()
    path = (root / scan_id).resolve()
    if root not in path.parents and path != root:
        raise ValueError("Invalid scan id")
    return path


def list_scans() -> list[dict[str, Any]]:
    root = scans_root()
    if not root.exists():
        return []
    scans: list[dict[str, Any]] = []
    for path in sorted(root.iterdir(), reverse=True):
        if not path.is_dir():
            continue
        if path.name == LATEST_SCAN_ID:
            continue
        meta_path = path / "scan.json"
        if not meta_path.exists():
            continue
        with open(meta_path, encoding="utf-8") as f:
            meta = json.load(f)
        meta["id"] = path.name
        meta["path_point_count"] = len(load_destinations(path.name))
        scans.append(meta)
    return scans


def load_scan_meta(scan_id: str) -> dict[str, Any]:
    path = _scan_dir(scan_id)
    meta_path = path / "scan.json"
    if not meta_path.exists():
        raise FileNotFoundError(scan_id)
    with open(meta_path, encoding="utf-8") as f:
        meta = json.load(f)
    meta["id"] = scan_id
    return meta


def load_floorplan(scan_id: str) -> dict[str, Any]:
    path = _scan_dir(scan_id)
    fp_path = path / "floorplan.json"
    if not fp_path.exists():
        raise FileNotFoundError(f"floorplan for {scan_id}")
    with open(fp_path, encoding="utf-8") as f:
        return json.load(f)


def _empty_floorplan_payload() -> dict[str, Any]:
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
        "upto_t": 0.0,
    }


def ensure_latest_scan() -> dict[str, Any]:
    root = ensure_scans_root()
    scan_path = root / LATEST_SCAN_ID
    meta_path = scan_path / "scan.json"
    if meta_path.exists():
        return load_scan_meta(LATEST_SCAN_ID)

    scan_path.mkdir(parents=True, exist_ok=True)
    created_at = datetime.now(timezone.utc).isoformat()
    meta = {
        "id": LATEST_SCAN_ID,
        "name": "Latest",
        "note": "",
        "created_at": created_at,
        "updated_at": created_at,
        "source_session_id": None,
        "scan_count": 0,
        "map_alignment": {"tx": 0.0, "ty": 0.0, "dyaw": 0.0},
    }
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
    with open(scan_path / "floorplan.json", "w", encoding="utf-8") as f:
        json.dump(_empty_floorplan_payload(), f)
    save_path(LATEST_SCAN_ID, [])
    return meta


def update_latest_scan(
    floorplan: dict[str, Any],
    *,
    source_session_id: str | None = None,
    odom_origin: dict[str, float] | None = None,
) -> dict[str, Any]:
    ensure_latest_scan()
    scan_path = _scan_dir(LATEST_SCAN_ID)
    meta_path = scan_path / "scan.json"
    with open(meta_path, encoding="utf-8") as f:
        meta = json.load(f)

    now = datetime.now(timezone.utc).isoformat()
    meta["updated_at"] = now
    meta["scan_count"] = floorplan.get("scan_count", 0)
    if source_session_id is not None:
        meta["source_session_id"] = source_session_id
    if odom_origin is not None:
        meta["odom_origin"] = odom_origin

    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
    with open(scan_path / "floorplan.json", "w", encoding="utf-8") as f:
        json.dump(floorplan, f)

    meta["id"] = LATEST_SCAN_ID
    return meta


def archive_latest_scan() -> str | None:
    """Move scans/latest to a dated directory. Returns archived id, or None if empty."""
    root = ensure_scans_root()
    scan_path = root / LATEST_SCAN_ID
    if not scan_path.is_dir():
        return None

    meta_path = scan_path / "scan.json"
    if not meta_path.exists():
        shutil.rmtree(scan_path)
        return None

    with open(meta_path, encoding="utf-8") as f:
        meta = json.load(f)

    scan_count = int(meta.get("scan_count") or 0)
    if scan_count == 0:
        return None

    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    archived_id = f"{ts}_scan"
    while (root / archived_id).exists():
        archived_id = f"{ts}_scan_{uuid.uuid4().hex[:4]}"

    dest = root / archived_id
    scan_path.rename(dest)

    meta["id"] = archived_id
    meta["name"] = archived_id
    meta["archived_from"] = LATEST_SCAN_ID
    meta["archived_at"] = datetime.now(timezone.utc).isoformat()
    with open(dest / "scan.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
    return archived_id


def reset_latest_scan() -> tuple[dict[str, Any], str | None]:
    """Archive the current latest scan (if any) and create a fresh scans/latest."""
    archived_id = archive_latest_scan()
    root = ensure_scans_root()
    latest_path = root / LATEST_SCAN_ID
    if latest_path.exists():
        shutil.rmtree(latest_path)
    return ensure_latest_scan(), archived_id


def restore_scan_to_latest(source_scan_id: str) -> tuple[dict[str, Any], str | None]:
    """Replace scans/latest with a copy of a historical scan."""
    if source_scan_id == LATEST_SCAN_ID:
        raise ValueError("Source is already the active latest scan")

    source_path = _scan_dir(source_scan_id)
    if not source_path.is_dir() or not (source_path / "scan.json").exists():
        raise FileNotFoundError(source_scan_id)

    root = ensure_scans_root()
    archived_id = archive_latest_scan()

    latest_path = root / LATEST_SCAN_ID
    if latest_path.exists():
        shutil.rmtree(latest_path)

    shutil.copytree(source_path, latest_path)

    meta_path = latest_path / "scan.json"
    with open(meta_path, encoding="utf-8") as f:
        meta = json.load(f)

    now = datetime.now(timezone.utc).isoformat()
    meta["id"] = LATEST_SCAN_ID
    meta["name"] = "Latest"
    meta["restored_from"] = source_scan_id
    meta["restored_at"] = now
    meta["updated_at"] = now
    meta.pop("archived_from", None)
    meta.pop("archived_at", None)

    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    meta["id"] = LATEST_SCAN_ID
    return meta, archived_id


def _normalize_points(points: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {"x": round(float(pt["x"]), 4), "y": round(float(pt["y"]), 4)}
        for pt in points
        if isinstance(pt, dict) and "x" in pt and "y" in pt
    ]


def load_path_data(scan_id: str) -> dict[str, list[dict[str, Any]]]:
    path = _scan_dir(scan_id)
    path_file = path / "path.json"
    if not path_file.exists():
        return {"route": [], "destinations": []}
    with open(path_file, encoding="utf-8") as f:
        data = json.load(f)
    route_raw = data.get("route")
    if not isinstance(route_raw, list):
        route_raw = data.get("points", [])
    route = _normalize_points(route_raw if isinstance(route_raw, list) else [])
    dest_raw = data.get("destinations")
    if isinstance(dest_raw, list) and dest_raw:
        destinations = _normalize_points(dest_raw)
    elif len(route) >= 2:
        destinations = [route[0], route[-1]]
    elif route:
        destinations = [route[-1]]
    else:
        destinations = []
    return {"route": route, "destinations": destinations}


def load_path(scan_id: str) -> list[dict[str, Any]]:
    return load_path_data(scan_id)["route"]


def load_destinations(scan_id: str) -> list[dict[str, Any]]:
    return load_path_data(scan_id)["destinations"]


def save_path(
    scan_id: str,
    route: list[dict[str, Any]],
    *,
    destinations: list[dict[str, Any]] | None = None,
) -> dict[str, list[dict[str, Any]]]:
    path = _scan_dir(scan_id)
    if not (path / "scan.json").exists():
        raise FileNotFoundError(scan_id)
    normalized_route = _normalize_points(route)
    normalized_destinations = _normalize_points(
        destinations if destinations is not None else normalized_route,
    )
    payload = {
        "route": normalized_route,
        "destinations": normalized_destinations,
        "points": normalized_route,
    }
    with open(path / "path.json", "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    return {"route": normalized_route, "destinations": normalized_destinations}


def delete_scan(scan_id: str) -> None:
    path = _scan_dir(scan_id)
    if not path.exists():
        raise FileNotFoundError(scan_id)
    shutil.rmtree(path)


def update_map_alignment(
    scan_id: str,
    tx: float,
    ty: float,
    *,
    dyaw: float = 0.0,
) -> dict[str, float]:
    """Store the rigid odom→map transform: p_map = R(dyaw)·p_odom + (tx, ty)."""
    path = _scan_dir(scan_id)
    meta_path = path / "scan.json"
    if not meta_path.exists():
        raise FileNotFoundError(scan_id)
    with open(meta_path, encoding="utf-8") as f:
        meta = json.load(f)
    alignment = {
        "tx": round(tx, 4),
        "ty": round(ty, 4),
        "dyaw": round(dyaw, 4),
    }
    meta["map_alignment"] = alignment
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
    return alignment
