"""Go2 dashboard FastAPI application."""

from __future__ import annotations

import asyncio
import signal
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from api import live, recordings, scans, settings
from live.manager import live_manager
from scan.store import ensure_scans_root


@asynccontextmanager
async def lifespan(_app: FastAPI):
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def _on_sigterm(signum, frame):
        live_manager.request_stop()
        if callable(previous_sigterm) and previous_sigterm not in (
            signal.SIG_DFL,
            signal.SIG_IGN,
        ):
            previous_sigterm(signum, frame)

    signal.signal(signal.SIGTERM, _on_sigterm)
    ensure_scans_root()
    yield
    live_manager.request_stop()
    await asyncio.to_thread(live_manager.shutdown)


def create_app() -> FastAPI:
    app = FastAPI(title="Go2 Dashboard API", version="0.1.0", lifespan=lifespan)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_methods=["*"],
        allow_headers=["*"],
    )
    app.include_router(live.router)
    app.include_router(recordings.router)
    app.include_router(scans.router)
    app.include_router(settings.router)
    return app


app = create_app()


def main() -> None:
    uvicorn.run(
        "app:app",
        host="0.0.0.0",
        port=8080,
        reload=True,
        timeout_graceful_shutdown=5,
    )
