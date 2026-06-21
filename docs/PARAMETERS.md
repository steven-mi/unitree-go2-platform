# Parameter reference

`config.yml` (repo root) is the **single source of truth** for every tunable
parameter in the backend. There are no hardcoded parameter defaults in the code:
each value is read straight from `config.yml` via `backend/src/config.py`, and a
missing key raises `ConfigError` at startup rather than silently falling back.

Two things interact with the environment:

1. **Deployment paths** (`PROJECT_ROOT`, `CONFIG_PATH`, `RECORDINGS_DIR`,
   `SCANS_DIR`) — environment variables, since they are infra wiring, not tuning,
   and they determine where `config.yml` itself is found.
2. **Env-var interpolation** — any string value in `config.yml` may reference an
   environment variable with `${VAR}` or `${VAR:-fallback}`, expanded when the
   file loads. E.g. `robot_ip: ${ROBOT_IP:-0.0.0.0}` uses `$ROBOT_IP`
   when set and falls back otherwise. Handy for pointing at a local robot (e.g.
   from Docker) without editing the file.

Editing notes:

- `nav.speed` and the `planner.*` clearance knobs are read **fresh on every
  point-and-go / replan**, so edits take effect without a backend restart.
- Everything else is read **once at import**; change it and restart the backend.
- The dashboard **Settings** panel rewrites `config.yml` when you save
  `robot_ip` / `aes_128_key`, which strips comments (PyYAML limitation).

---

## Robot connection (top-level keys)

All string values support `${VAR}` / `${VAR:-fallback}` interpolation.

| Key | Default | Purpose |
| --- | --- | --- |
| `robot_ip` | `${ROBOT_IP:-0.0.0.0}` | Go2 IP for the WebRTC connection. Settable in dashboard Settings or via `$ROBOT_IP`. |
| `aes_128_key` | _(empty)_ | Per-device AES-128 key (32 hex chars). Required on Go2 ≥ 1.1.15 / G1 ≥ 1.5.1. Fetch with `unitree-fetch-aes-key`. |

### Deployment paths (env only)

| Env var | Default | Purpose |
| --- | --- | --- |
| `PROJECT_ROOT` | repo root | Base path for `config.yml`. |
| `CONFIG_PATH` | `<root>/config.yml` | Explicit config file path. |
| `RECORDINGS_DIR` | `backend/recordings` or `../recordings` | Session recordings store. |
| `SCANS_DIR` | `../scans` | Saved scans store. |

---

## Floor-plan builder (`floorplan:`)

Drives `FloorPlanBuilder` in `backend/src/floorplan/builder.py`. Used by both live
(`live/manager.py`) and replay (`replay/session.py`).

The builder keeps an integer **evidence score** per cell: each scan a cell is hit
as tall structure adds `+1`, each scan a ray passes through it subtracts `1`. A
cell is a **wall** at `score ≥ hits_to_confirm` and **free** at
`score ≤ -clears_to_free`; the score saturates at those two bounds. This replaces
the old log-odds knobs (`l_occ`, `l_free`, `l_clamp_*`, `occ_threshold`,
`free_threshold`) with two intuitive counts.

| Key | Default | Effect |
| --- | --- | --- |
| `revision` | `13` | Cache revision; bump to invalidate cached floor plans and force regeneration after algorithm/parameter changes. |
| `resolution` | `0.05` m | Grid cell size (5 cm). |
| `z_min` | `0.10` m | Floor clip — returns below this height are ignored so the floor is never mapped as a wall. |
| `z_max` | `1.6` m | Ceiling clip — returns above this height are ignored. |
| `wall_height_min` | `0.2` m | A cell earns wall evidence only if a return reaches this height. Must sit well above `z_min`; a value ≤ `z_min` disables the gate and paints floor clutter as walls. |
| `wall_min_points` | `2` | Tall returns (above `wall_height_min`) required in one `(x, y)` column before it earns wall evidence. Real walls stack many voxels vertically; lone reflections/passing heads do not. Set to `1` to disable; raise to reject more noise at the risk of dropping sparse/distant walls. Especially important when `hits_to_confirm` is low, since one spurious frame would otherwise paint a permanent wall. |
| `hits_to_confirm` | `1` | Net wall observations before a cell is drawn as a wall. Higher ⇒ more noise rejection, slower to map. |
| `clears_to_free` | `2` | Net ray clears before a cell is treated as free space. |
| `max_ray_m` | `14.0` m | Max ray-cast length for free-space clearing. |
| `cleanup_radius` | `2` cells | Morphological close radius that bridges small wall gaps; interior open uses `cleanup_radius // 2`. |
| `min_wall_cells` | `5` | Drop wall connected-components smaller than this (speckle). |
| `path_clearance_m` | `0.30` m | Radius around the walked path / robot to clear false walls. |
| `margin_m` | `0.6` m | Padding when expanding grid bounds for new returns. |
| `max_dim` | `2400` cells | Max grid dimension; larger maps are downsampled. |

Replay **resets and rebuilds** the builder if you scrub backward in time
(`t < synced_t - 0.05`). The programmatic `build_floor_plan_result()` also accepts
`scan_stride` (default `1`) and `crop` (default `True`), which are call arguments,
not config keys.

---

## Path planning (`planner:`)

A* routing in `backend/src/route_planning/planner.py`.

The pipeline is intentionally simple: **single hard inflation pass** (walls grow
by exactly `robot_radius_m`, so the path may go anywhere the dog physically fits)
→ **A\* with a mild centering penalty** (bias off walls) → **line-of-sight
string-pull** (straighten the 8-connected staircase into linear segments) →
**slight corner rounding**. This replaced the old multi-radius fallback ladder
(`safety_margin_m`, `min_robot_radius_m`, `radius_fallback_steps`). Corner
rounding is applied once to the full stitched route in `plan_route_on_floorplan`,
so the live follower drives that geometry directly (the manager no longer
re-simplifies it).

### Clearance & centering (read fresh per plan)

| Key | Default | Effect |
| --- | --- | --- |
| `robot_radius_m` | `0.25` m | **(Hard footprint)** The single wall-inflation radius. Walls are grown by exactly this and the path never enters anything tighter, so it sets the minimum gap the dog will attempt (~`2 × robot_radius_m`) and the distance the planned path keeps the body off every wall. Must exceed the Go2 half-width (~`0.16` m) with margin or the dog grazes walls at pinch points — the driving safety cone now trusts the map (ignores known walls), so this radius is the only thing holding the body off them. Lower toward `~0.22` only if a known-tight doorway becomes unreachable. |
| `centering_weight` | `10.0` | **(Soft centering)** Penalty added to A* edge cost right at a wall. Higher ⇒ the path works harder to stay away from walls / centred; `0` disables centering (shortest path). |
| `centering_clearance_m` | `0.8` m | Distance from walls beyond which there's no extra reward. The penalty ramps **linearly** from `centering_weight` at the wall to `0` at this distance: `weight * clamp(1 - clearance / centering_clearance_m, 0, 1)`. |
| `corner_smoothing_iters` | `2` | Chaikin corner-rounding passes applied to the stitched route so the dog walks more naturally. `0` disables (hard corners). More ⇒ smoother and denser. Kept light. |
| `corner_smoothing_ratio` | `0.2` | Fraction of each edge trimmed per corner (`0..0.5`). Larger ⇒ rounder corners. Each cut is rejected if it would leave the inflated free space, so smoothing never costs clearance below `robot_radius_m`. |

Because the penalty keeps decreasing across the whole `0 .. centering_clearance_m`
range (unlike an exponential that saturates a few cm off the wall), A* genuinely
maximizes wall distance: in corridors narrower than `2 × centering_clearance_m`
the cheapest route is the centre line; in wider spaces it simply holds that much
clearance. The penalty is always ≥ 0, so the octile heuristic stays admissible
and A* remains optimal w.r.t. the penalised cost.

After A*, a **clearance-preserving string-pull** straightens the 8-connected
staircase: it drops a waypoint only when the straight shortcut stays inside the
inflated free space **and** keeps at least as much wall clearance as the A*
sub-path it replaces. So straight corridors collapse to single segments while
centred curves around corners are kept (never cut short against a wall), and
smoothing never costs clearance below `robot_radius_m`.

### Search

| Key | Default | Effect |
| --- | --- | --- |
| `max_snap_radius` | `40` cells (~2 m) | Max search radius when snapping a start/goal to the nearest free cell. |

---

## Obstacle detection (`obstacle:`)

Live obstacles come from the connected robot's **lidar** and feed two consumers:
the route-planning mask (`lidar_obstacle_mask`, which fuses extra blocked cells
into the A* grid) and the driving safety pause in `navigation.py` (a lidar-only
stop that backs up the Go2 firmware avoidance). Only active while the robot is
**connected** and live lidar is available; offline planning on a saved scan uses
the saved walls only.

### Detection geometry (shared)

| Key | Default | Effect |
| --- | --- | --- |
| `z_min` / `z_max` | `0.12` / `1.20` m | Height band — returns outside it (floor / ceiling / sparse noise) are ignored. |
| `front_half_angle_deg` | `75` deg | Forward field-of-view half-angle relative to the dog's heading; returns to the side or behind are ignored. `180` = detect all around. Applied to the planning mask when the robot pose is known. |
| `self_clear_m` | `0.35` m | Radius around the robot never treated as an obstacle (its own footprint). |

### Route-planning mask

| Key | Default | Effect |
| --- | --- | --- |
| `dilate_cells` | `1` | Thicken raw returns into a solid blob before blocking. |
| `wall_guard_cells` | `3` | Discard returns within this many cells of a known wall (localization slop). |
| `min_cells` | `4` | Drop obstacle clusters smaller than this (speckle). |

### Driving pause (lidar-only, secondary to firmware avoidance)

| Key | Default | Effect |
| --- | --- | --- |
| `stop_m` | `0.75` m | Halt when an obstacle is closer than this along the forward / path cone. |
| `clear_m` | `0.90` m | Resume only after clearance exceeds this (hysteresis). |
| `clear_s` | `0.5` s | Sustained-clear time required before resuming. |
| `cone_half_m` | `0.38` m | Half-width of the forward stop cone. |
| `body_min_x` | `0.15` m | Ignore returns inside the robot body envelope. |
| `min_points` | `5` | Minimum lidar hits to trigger a stop (sparse-data tolerant). |
| `forward_block_min_vx` | `0.04` m/s | Only check when the tracker intends forward motion. |
| `wait_poll_s` | `0.05` s | Poll interval while paused. |
| `pause_max_s` | `60.0` s | Max wait during an obstacle pause. |
| `final_approach_margin_m` | `0.10` m | On the **last** waypoint the stop cone is shrunk to `distance_to_goal - this` (clamped to `body_min_x`), so a wall behind a goal placed closer than `stop_m` doesn't latch the pause and prevent arrival. Obstacles nearer than the goal still stop the dog. The planner already proved the goal reachable. |

---

## Navigation motion (`nav:`)

Closed-loop path following in `backend/src/live/navigation.py`.

| Key | Default | Effect |
| --- | --- | --- |
| `speed` | `0.25` m/s | Point-and-go cruise target (read fresh each run, clamped to `min_speed`..`max_speed`). |
| `min_speed` | `0.05` m/s | Lower clamp for `speed`. |
| `max_speed` | `0.6` m/s | Upper clamp for `speed`. |
| `turn_speed` | `0.85` rad/s | Max yaw rate while tracking / turning in place. |
| `move_hz` | `50.0` Hz | Command/control loop rate. |
| `lookahead_m` | `0.85` m | Pure-pursuit lookahead — larger = smoother at higher speed. |
| `slow_dist_m` | `0.45` m | Distance over which speed ramps down near a waypoint. |
| `waypoint_reached_m` | `0.40` m | Radius to consider a waypoint reached. |
| `kp_yaw` | `2.0` | Proportional gain on heading error. |
| `pose_max_age_s` | `0.4` s | Stop and wait if the localization pose is older than this (resumes automatically when fresh pose returns). Prevents the dog circling when odom can't keep up with speed. Steering also holds the last setpoint between pose updates rather than re-deriving a correction every control tick. |
| `stuck_timeout_s` | `60.0` s | Abort a leg if no waypoint progress for this long. |
| `stuck_recover_s` | `10.0` s | Skip ahead a waypoint after this with no progress. |
| `turn_in_place_rad` | `0.85` rad | Yaw error above which the dog starts rotating in place. |
| `turn_resume_rad` | `0.20` rad | Hysteresis: once an in-place turn starts it keeps rotating until the heading error drops to this, so the turn always finishes instead of the dog driving off badly misaligned (or stopping half-turned). |
| `wireless_vx_full` | `0.35` m/s | Intent forward speed mapped to full stick (`ly = 1.0`). |
| `wireless_vy_full` | `0.35` m/s | Intent lateral speed mapped to full stick (`lx = 1.0`). |
| `wireless_vyaw_full` | `2.5` rad/s | Intent yaw rate mapped to full stick (`rx = 1.0`). |

Obstacle stop / resume behaviour while driving lives in the [`obstacle:`](#obstacle-detection-obstacle) section above.

When replanning finds no route around a blockage, the cockpit keeps the mission
active, halts the dog, and retries until a path opens or the user presses Stop.

---

## Manual teleop (`teleop:`)

Manual drive caps in `backend/src/live/teleop.py`.

| Key | Default | Effect |
| --- | --- | --- |
| `max_vx` | `0.6` m/s | Forward/back cap (keyboard / joystick). |
| `max_vy` | `0.25` m/s | Lateral (strafe) cap. |
| `max_vyaw` | `0.85` rad/s | Turn cap. |

---

## HTTP query parameters (floorplan endpoints)

Exposed on all three floorplan endpoints (live, recording, scan):

| Param | Type | Purpose |
| --- | --- | --- |
| `t` | float | Build map using data **up to this session time** (seconds). Omit = full session / live elapsed time. |
| `x` | float | Robot X for corridor clearing near current position. |
| `y` | float | Robot Y for corridor clearing. |
| `lidar_seq` | int | Re-ingest a specific lidar frame (for live sync with the 3D view). |

Example:

```
GET /api/recordings/abc123/floorplan?t=42.5&x=1.2&y=3.4&lidar_seq=87
```
