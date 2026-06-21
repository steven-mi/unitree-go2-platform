

# Go2 Dashboard Backend

FastAPI service for the Unitree Go2 dashboard: live robot telemetry over WebRTC, session recording, and replay of saved recordings (lidar, video, floor plans).

Part of the [unitree_dog](../) monorepo. Depends on the [unitree_webrtc_connect](../unitree_webrtc_connect) git submodule.

## Layout

```
backend/
  pyproject.toml      # uv project
  src/
    app.py            # FastAPI app + entrypoint
    config.py         # config.yml + env-based paths
    api/              # HTTP routes
    live/             # WebRTC connection + live buffers
    replay/           # load and query saved sessions
    parsing/          # decode robot topic payloads
    floorplan/        # lidar → 2D floor plan
    recording/        # session recorder library
    domain/           # shared types
```

## Prerequisites

- Python 3.12+
- [uv](https://docs.astral.sh/uv/)
- Submodule initialized: `git submodule update --init --recursive`

## Local development

From the repo root:

```bash
uv sync --project backend
uv run --project backend uvicorn app:app --reload --host 0.0.0.0 --port 8080
```

Or use the script entrypoint:

```bash
uv run --project backend go2-dashboard
```

API docs: [http://localhost:8080/docs](http://localhost:8080/docs)

## Docker

From the repo root (starts backend + frontend):

```bash
docker compose up --build
```

The backend container runs `uv run uvicorn app:app` with hot reload. Recordings are mounted at `/app/recordings`. Robot connection settings live in `config.yml` at the repo root (mounted into the container).

## Configuration

Robot IP and AES key are stored in `config.yml` at the repo root (or edited via **Settings** in the dashboard):

```yaml
robot_ip: ${ROBOT_IP}
aes_128_key: ""   # 32 hex chars; required on Go2 >= 1.1.15
```

`config.yml` at the repo root is the single source of truth for every tunable
parameter. See `[docs/PARAMETERS.md](../docs/PARAMETERS.md)` for the full reference.

## Environment variables


| Variable         | Default                     | Description                 |
| ---------------- | --------------------------- | --------------------------- |
| `RECORDINGS_DIR` | `../recordings` (repo root) | Directory of saved sessions |
| `SCANS_DIR`      | `../scans` (repo root)      | Directory of saved scans    |


## API overview


| Prefix              | Purpose                                                                                       |
| ------------------- | --------------------------------------------------------------------------------------------- |
| `/api/live/*`       | Connect/disconnect, live frames, lidar (binary), floor plan, recording, teleop WS, navigation |
| `/api/scans/*`      | Saved maps, path planning, localization, latest scan sync                                     |
| `/api/recordings/*` | List sessions, replay frames, lidar (binary), floor plan, video files                         |
| `/api/settings`     | Read/update `config.yml` (robot IP, AES key)                                                  |


Floor plans are built on demand from lidar scans up to a playback timestamp.