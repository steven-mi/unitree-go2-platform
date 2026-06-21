"""Shared domain types."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class IndexEntry:
    recv_t: float
    payload: dict[str, Any]
