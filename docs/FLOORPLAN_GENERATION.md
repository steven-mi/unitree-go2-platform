# Floor Plan Generation

How the dashboard builds a 2D top-down map from lidar scans and odometry.

Core implementation: `backend/src/floorplan/builder.py` (`FloorPlanBuilder`).

The builder is an **evidence-count occupancy grid**, not a binary "ever-hit =
wall" splat:

- **Score cells** — each cell keeps a small integer score: `+1` per scan it is hit
  as tall structure, `-1` per scan a ray passes through it. A cell is a wall at
  `score ≥ hits_to_confirm` and free at `score ≤ -clears_to_free`. Requiring
  several confirming hits rejects stray points, so noise and dynamic obstacles
  decay while real structure sharpens into thin walls.
- **Ray casting** — every scan clears the free cells between the robot and each
  wall return, carving clean interiors and keeping walls one cell thick.
- **Floor clip + height gate** — only returns above the floor that reach real wall
  height earn occupied evidence, so the floor is never mapped as a wall.

---

## Entry points

| Source | API | Backend |
| ------ | --- | ------- |
| Live session | `GET /api/live/floorplan` | `LiveManager.build_floor_plan()` |
| Recording replay | `GET /api/recordings/{id}/floorplan` | `SessionReplay.build_floor_plan()` |
| Saved scan | `GET /api/scans/{id}/floorplan` | Same replay path |

The frontend (`FloorPlanView`) polls these endpoints during live/replay and renders the returned grids.

---

## Pipeline

```
Lidar scans ──► ingest_points() ──► evidence-count occupancy grid
Odom samples ──► append_path()  ──┘   (ray cast from robot pose)
                                        │
                                        ▼
                              _finalize_grid()
                                        │
                                        ▼
                         interior (zones) + walls + path
                                        │
                                        ▼
                         floor_plan_to_api() → base64 grids
```

### 1. Ingest lidar scans

For each scan up to time `t`:

1. Load the point cloud (`float32` N×3: x, y, z) — the Go2 streams a *rolling local
   voxel map* in the odom frame that follows the robot.
2. Look up nearest odom for the robot pose (used as the ray-cast origin **and** the
   path trail).
3. Call `ingest_points(pts, robot_x, robot_y)`.

Inside `ingest_points`:

- Keep points with **z ∈ [0.10, 1.6]** — `z_min` clips the floor so flat floor
  returns never become walls. Use **raw lidar x/y** as map coordinates (Go2 lidar
  is already odom-oriented; pose rotation is **not** applied).
- A cell is a **wall return this frame** when its column holds **at least
  `wall_min_points` (2)** returns reaching **≥ `wall_height_min` (0.2 m)**, measured
  *per frame* (a cumulative test would be inflated by body-height / tilt drift and
  fake a floor wall). Counting tall returns per column — real walls stack many
  voxels vertically, while a lone reflection or a passing head is a single point —
  rejects stray points *before* they earn evidence. This matters because
  `hits_to_confirm` can be as low as 1, so a single spurious frame would otherwise
  paint a permanent wall.
- **Ray cast** from the robot cell to each wall return: cells along the ray get a
  free observation (`score -= 1`), the wall endpoint gets a wall observation
  (`score += 1`). A ray never clears another wall it grazes through (thin walls
  would erode away).
- The score saturates at `[-clears_to_free, hits_to_confirm]`.

If no robot pose is available (e.g. saved-scan hydration), the frame falls back to
occupied-only evidence with no ray clearing.

### 2. Ingest robot path

Every other odom sample appends `(x, y)` to `path_pts`. The path is used to:

- Seed flood-fill for explored interior.
- Clear false walls along the walked corridor.

### 3. Optional live frame injection

If `lidar_seq` is passed, that specific scan is re-ingested with `count_scan=False` (does not increment scan count) but still contributes geometry. Pose can be overridden via `x`/`y` query params.

### 4. Finalize grid (`_finalize_grid`)

Turns raw hits into two boolean maps:

| Output | API field | Meaning |
| ------ | --------- | ------- |
| Interior | `zones_b64` | Explored, navigable free space |
| Outline | `walls_b64` | Wall/obstacle cells |

Steps:

1. **Wall / free classification** — `walls = score ≥ hits_to_confirm`,
   `free = score ≤ -clears_to_free`. Occupied evidence is only ever added to
   per-frame wall columns, so the score sign alone separates walls from free.
2. **Corridor clearing** — cells within **0.30 m** of the walked path (and current
   robot position if `x`/`y` given) are removed from walls and forced free. Cleans
   smear where the dog actually drove.
3. **Wall cleanup** — morphological `close` (`cleanup_radius`, **2 cells**) bridges
   small gaps, then connected components smaller than `min_wall_cells` (**5**) are
   dropped.
4. **Interior** — flood-fill free space from path seeds (+ robot position), fill
   enclosed holes, subtract walls, then a morphological `open`
   (`cleanup_radius // 2`) smooths the result.
5. **Crop** — tight bounding box around interior + walls with 0.35 m padding.
6. **Serialize** — `floor_plan_to_api()` base64-encodes `zones` (uint8, 1 = interior) and `walls` (uint8, 1 = wall).

---

## HTTP query parameters

See the [parameter reference](PARAMETERS.md#http-query-parameters-floorplan-endpoints)
for the full list (`t`, `x`, `y`, `lidar_seq`).

### Frontend fetch behavior

`FloorPlanView` / `fetchFloorPlan()` in `frontend/src/api.ts`:

- **Paused / scrubbing:** rebuild on every `t` or `lidarSeq` change.
- **Playing:** debounced ~120 ms, triggered by new `lidarSeq` (not every animation-frame `t` tick).
- **Saved scans:** load once at `t=0`, no pose.

---

## Builder parameters (code-level, not HTTP)

All `FloorPlanBuilder` knobs (and the replay-only `scan_stride` / `crop` options)
are documented in the [parameter reference](PARAMETERS.md#floor-plan-builder).
Live and replay construct it as `FloorPlanBuilder(resolution=0.05)`, so every
other knob uses its dataclass default.

Replay **resets and rebuilds** the builder if you scrub backward in time (`t < synced_t - 0.05`).

---

## API response

```json
{
  "width": 120,
  "height": 80,
  "origin_x": -1.5,
  "origin_y": -2.0,
  "resolution": 0.05,
  "scan_count": 42,
  "threshold": 50,
  "zone_count": 1,
  "map_rotation": 0.0,
  "zones_b64": "...",
  "walls_b64": "...",
  "path": [{"x": 0.1, "y": 0.2}, ...],
  "upto_t": 42.5
}
```

| Field | Meaning |
| ----- | ------- |
| `origin_x` / `origin_y` | World coordinates of grid cell (0, 0). |
| `zones_b64` | Base64-encoded `width × height` uint8 grid; 1 = explored interior. |
| `walls_b64` | Base64-encoded `width × height` uint8 grid; 1 = wall. |
| `path` | Odometry trail as `{x, y}` samples. |
| `threshold` | Wall height gate used, in cm (0.5 m → `50`). |
| `upto_t` | Session time the map was built up to. |

An empty 1×1 map is returned when no wall-height data exists yet.

---

## Design choices

1. **Lidar frame = map frame** — walls are drawn in the same coordinate system as the 3D point cloud panel, using the Go2's odom-frame voxel cloud + `ROBOTODOM` pose. No extra pose transform or scan matching is applied.
2. **Evidence-count occupancy** — per-cell hit/clear counting + ray casting, so walls sharpen once several scans confirm them and transient/dynamic obstacles decay instead of sticking forever.
3. **Floor clip + height gate** — occupancy ignores floor-height returns and only counts tall vertical structure, so the floor and low clutter never become walls.
4. **Path-based cleaning** — the walked corridor is carved out of walls to handle residual smear.
5. **Incremental + cached** — the builder persists across requests; only new scans since last `t` are ingested. Live is append-only; replay can rewind.
6. **`FLOOR_PLAN_REV`** in `backend/src/config.py` — bumping this invalidates cached builders when algorithm defaults change (currently `12`).

### Known limits / next levers

The remaining wall thickness and waviness come from the **data source**, not pose
drift. Measured on real recordings:

- The Go2's `ROBOTODOM` (`rt/utlidar/robot_pose`) is lidar-inertial odometry and is
  already accurate — incremental scan-to-map matching gives a net correction of
  **< 1 m over a whole session**, and ~90% of scans need *zero* correction.
- Naive incremental scan matching is therefore **not worth it**: translation-only
  correction produces a visually identical map, and adding a yaw search **diverges**
  (it over-rotates the sparse scans, accumulating tens of degrees and badly warping
  the map). This was prototyped and rejected.

So the real lever for sharper walls is **denser/cleaner lidar data**, not pose
correction: `rt/utlidar/voxel_map_compressed` is sparse. The Unitree mapping stack
(`rt/uslam/frontend/cloud_world_ds`, available only while the app's SLAM is running)
is the denser source if higher fidelity is ever required.

---

## Downstream use

Floor plans feed into:

- **Path planning** — `backend/src/route_planning/planner.py` (`plan_path_on_floorplan`).
- **Scan localization** — `backend/src/localization/matcher.py`.

### Planner clearance (keeping the dog off walls)

A* runs on the wall grid inflated by the robot's inscribed radius, plus a hard
safety margin and a soft proximity penalty that steers paths toward corridor
centres. All knobs are in the
[parameter reference](PARAMETERS.md#path-planning-clearance).

Route planning also fuses **live lidar obstacles** into the grid so the dog routes
around (or stops for) obstacles that appear after the map was built — see
[live obstacle avoidance](PARAMETERS.md#live-obstacle-avoidance-dynamic-re-routing).
