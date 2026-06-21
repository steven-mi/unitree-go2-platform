"""Load and index recorded sessions from disk."""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any

from config import recordings_root
from domain.models import IndexEntry
from parsing.io import read_jsonl
from parsing.rpc import load_rpc_snapshots, load_services
from replay.session import SessionReplay


def normalize_tags(tags: Any) -> list[str]:
    if not isinstance(tags, list):
        return []
    seen: set[str] = set()
    out: list[str] = []
    for tag in tags:
        if not isinstance(tag, str):
            continue
        cleaned = tag.strip()
        if not cleaned:
            continue
        key = cleaned.casefold()
        if key in seen:
            continue
        seen.add(key)
        out.append(cleaned)
    return out


def read_session_tags(path: Path, manifest: dict[str, Any] | None = None, session: dict[str, Any] | None = None) -> list[str]:
    manifest = manifest if manifest is not None else {}
    session = session if session is not None else {}
    if "tags" in manifest:
        return normalize_tags(manifest.get("tags"))
    if "tags" in session:
        return normalize_tags(session.get("tags"))
    session_path = path / "session.json"
    if session_path.exists():
        with open(session_path, encoding="utf-8") as f:
            return normalize_tags(json.load(f).get("tags"))
    return []


def update_session_tags(session_id: str, tags: list[str], root: Path | None = None) -> list[str]:
    root = (root or recordings_root()).resolve()
    path = (root / session_id).resolve()
    if not path.is_dir() or root not in path.parents:
        raise FileNotFoundError(session_id)

    normalized = normalize_tags(tags)
    for filename in ("session.json", "manifest.json"):
        meta_path = path / filename
        if not meta_path.exists():
            continue
        with open(meta_path, encoding="utf-8") as f:
            data = json.load(f)
        data["tags"] = normalized
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    return normalized


def delete_session(session_id: str, root: Path | None = None) -> None:
    root = (root or recordings_root()).resolve()
    path = (root / session_id).resolve()
    if not path.is_dir() or root not in path.parents:
        raise FileNotFoundError(session_id)
    if path.name == "latest":
        raise ValueError("Cannot delete the 'latest' alias")

    shutil.rmtree(path)

    latest = root / "latest"
    if latest.is_symlink() and not latest.exists():
        latest.unlink()


def list_sessions(root: Path | None = None) -> list[dict[str, Any]]:
    root = root or recordings_root()
    if not root.exists():
        return []
    sessions = []
    for path in sorted(root.iterdir(), reverse=True):
        if not path.is_dir() or path.name == "latest":
            continue
        manifest_path = path / "manifest.json"
        session_path = path / "session.json"
        manifest: dict[str, Any] = {}
        session: dict[str, Any] = {}
        meta: dict[str, Any] = {"id": path.name, "path": str(path)}
        if manifest_path.exists():
            with open(manifest_path, encoding="utf-8") as f:
                manifest = json.load(f)
            meta.update({
                "note": manifest.get("note") or "",
                "duration_s": manifest.get("duration_s"),
                "created_at": manifest.get("created_at"),
                "lidar_count": manifest.get("streams", {}).get("lidar", {}).get("count", 0),
                "video_count": manifest.get("streams", {}).get("video", {}).get("count", 0),
                "interrupted": manifest.get("interrupted", False),
            })
        if session_path.exists():
            with open(session_path, encoding="utf-8") as f:
                session = json.load(f)
            if not manifest_path.exists():
                meta.update({
                    "note": session.get("note") or "",
                    "created_at": session.get("created_at"),
                })
        if not manifest_path.exists() and not session_path.exists():
            continue
        meta["tags"] = read_session_tags(path, manifest, session)
        sessions.append(meta)
    return sessions


def load_session(session_id: str, root: Path | None = None) -> SessionReplay:
    root = (root or recordings_root()).resolve()
    path = (root / session_id).resolve()
    if not path.is_dir() or root not in path.parents:
        raise FileNotFoundError(session_id)

    manifest = {}
    mf = path / "manifest.json"
    if mf.exists():
        with open(mf, encoding="utf-8") as f:
            manifest = json.load(f)

    session_meta = {}
    sp = path / "session.json"
    if sp.exists():
        with open(sp, encoding="utf-8") as f:
            session_meta = json.load(f)

    video = [IndexEntry(r["recv_t"], r) for r in read_jsonl(path / "video" / "index.jsonl")]
    lidar = [IndexEntry(r["recv_t"], r) for r in read_jsonl(path / "lidar" / "index.jsonl")]
    odom = [IndexEntry(r["recv_t"], r) for r in read_jsonl(path / "topics" / "ROBOTODOM.jsonl")]
    sport = [IndexEntry(r["recv_t"], r) for r in read_jsonl(path / "topics" / "LF_SPORT_MOD_STATE.jsonl")]
    battery = [IndexEntry(r["recv_t"], r) for r in read_jsonl(path / "topics" / "LOW_STATE.jsonl")]
    ulidar_state = [IndexEntry(r["recv_t"], r) for r in read_jsonl(path / "topics" / "ULIDAR_STATE.jsonl")]
    uwb = [IndexEntry(r["recv_t"], r) for r in read_jsonl(path / "topics" / "UWB_STATE.jsonl")]
    multiple_state = [IndexEntry(r["recv_t"], r) for r in read_jsonl(path / "topics" / "MULTIPLE_STATE.jsonl")]
    audio_hub = [IndexEntry(r["recv_t"], r) for r in read_jsonl(path / "topics" / "AUDIO_HUB_PLAY_STATE.jsonl")]

    all_entries = video + lidar + odom + sport + battery + ulidar_state + uwb + multiple_state + audio_hub
    if not all_entries:
        raise ValueError(f"session {session_id} has no replay data")

    t0 = min(e.recv_t for e in all_entries)
    t_end = max(e.recv_t for e in all_entries)
    duration = manifest.get("duration_s") or (t_end - t0)

    return SessionReplay(
        session_id=session_id,
        root=path,
        manifest=manifest,
        session_meta=session_meta,
        t0=t0,
        duration=float(duration),
        video=video,
        lidar=lidar,
        odom=odom,
        sport=sport,
        battery=battery,
        ulidar_state=ulidar_state,
        uwb=uwb,
        multiple_state=multiple_state,
        audio_hub=audio_hub,
        rpc=load_rpc_snapshots(path),
        services=load_services(path),
    )
