"""Request/response models shared across API routers."""

from __future__ import annotations

from pydantic import BaseModel


class PathPointBody(BaseModel):
    x: float
    y: float
