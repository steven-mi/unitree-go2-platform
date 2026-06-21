# Go2 Dashboard

Browser dashboard for the [Unitree Go2](https://www.unitree.com/go2) — drive, map, navigate, and replay over WebRTC on your local network.

## Features

- **Live** — synced camera, 3D lidar, floor plan, and telemetry, with full session recording
- **Joystick** — low-latency keyboard teleop, walk modes, stances, and tricks
- **Scan** — walk the dog through a room to build and save a map
- **Point & Go** — place destinations on a saved map and send the dog along the route
- **Recordings** — replay sessions on a synced timeline (video, lidar, pose, floor plan)
- **Settings** — set the robot IP and optional AES key in `config.yml`

Point & Go drives the dog with open-loop turn/forward WebRTC commands planned by the dashboard backend.

## Quick start

This repo uses [unitree_webrtc_connect](https://github.com/steven-mi/unitree_webrtc_connect) as a git submodule.

```bash
git clone --recurse-submodules https://github.com/steven-mi/unitree-go2-platform.git
cd unitree-go2-platform
docker compose up --build
```

Open [http://localhost:5173](http://localhost:5173), set your Go2's IP in **Settings**, then connect from any live page.

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

## Configuration

All tunable parameters live in `[config.yml](config.yml)` at the repo root — it is the single source of truth, with no hidden code defaults. The two keys you normally touch:


| Key           | Purpose                                                                                                           |
| ------------- | ----------------------------------------------------------------------------------------------------------------- |
| `robot_ip`    | Go2 IP for the WebRTC connection. Defaults to `${ROBOT_IP}`; also settable from the dashboard Settings.           |
| `aes_128_key` | Per-device AES-128 key (32 hex chars). Required on Go2 ≥ 1.1.15 / G1 ≥ 1.5.1. Fetch with `unitree-fetch-aes-key`. |


The remaining sections (`floorplan`, `planner`, `nav`, `teleop`) tune mapping, path planning, navigation, and teleop. See [docs/PARAMETERS.md](docs/PARAMETERS.md) for the full reference.

> The dashboard Settings panel rewrites `config.yml` when you save `robot_ip` / `aes_128_key`, which strips comments — the annotated reference lives in `docs/PARAMETERS.md`.

## Stack


| Layer      | Tech                                                                                    |
| ---------- | --------------------------------------------------------------------------------------- |
| Frontend   | React + TypeScript (Vite)                                                               |
| Backend    | FastAPI — WebRTC, recording, replay, floor plans, navigation                            |
| Robot link | [unitree_webrtc_connect](https://github.com/steven-mi/unitree_webrtc_connect) submodule |


The frontend talks to the backend over REST (and WebSocket for teleop); the backend owns all robot I/O. Saved data lives in `recordings/` and `scans/`.

More detail: [backend/README.md](backend/README.md), [frontend/README.md](frontend/README.md), [docs/GO2_WEBRTC_REFERENCE.md](docs/GO2_WEBRTC_REFERENCE.md), [docs/PARAMETERS.md](docs/PARAMETERS.md), [docs/FLOORPLAN_GENERATION.md](docs/FLOORPLAN_GENERATION.md).

## Update unitree_webrtc_connect

```bash
cd unitree_webrtc_connect
git fetch upstream && git merge upstream/master && git push origin master
cd ..
git submodule update --remote unitree_webrtc_connect
git add unitree_webrtc_connect && git commit -m "Bump unitree_webrtc_connect submodule"
```

