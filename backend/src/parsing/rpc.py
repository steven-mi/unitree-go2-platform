"""RPC snapshot loading and summarization."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Optional

from parsing.io import read_jsonl
from parsing.topics import row_data


def summarize_rpc(name: str, raw: dict[str, Any]) -> Optional[dict[str, Any]]:
    data = raw.get("data", {})
    if not isinstance(data, dict):
        return None

    header = data.get("header", {})
    status = header.get("status", {}) if isinstance(header, dict) else {}
    code = status.get("code") if isinstance(status, dict) else None
    out: dict[str, Any] = {}
    if code is not None:
        out["status_code"] = int(code)

    inner = data.get("data")
    if isinstance(inner, str) and inner:
        try:
            parsed = json.loads(inner)
        except json.JSONDecodeError:
            parsed = inner
        if isinstance(parsed, dict):
            out.update(parsed)
        else:
            out["data"] = parsed
    elif isinstance(inner, dict):
        if inner.get("__omitted__") == "bytes":
            out["bytes"] = inner.get("len")
        else:
            out.update(inner)

    return out or None


def load_rpc_snapshots(path: Path) -> dict[str, Any]:
    rpc_dir = path / "rpc"
    if not rpc_dir.exists():
        return {}
    snapshots: dict[str, Any] = {}
    for rpc_path in sorted(rpc_dir.glob("*.json")):
        with open(rpc_path, encoding="utf-8") as f:
            raw = json.load(f)
        summary = summarize_rpc(rpc_path.stem, raw)
        if summary:
            snapshots[rpc_path.stem] = summary
    return snapshots


def load_services(path: Path) -> list[dict[str, Any]]:
    rows = read_jsonl(path / "topics" / "SERVICE_STATE.jsonl")
    if not rows:
        return []
    data = row_data(rows[0])
    if not isinstance(data, list):
        return []
    services = []
    for item in data:
        if not isinstance(item, dict):
            continue
        services.append({
            "name": str(item.get("name", "")),
            "status": int(item["status"]) if isinstance(item.get("status"), (int, float)) else None,
            "version": str(item.get("version", "")),
        })
    return services
