# Go2 Pro — Interfaces

How you talk to the dog over WebRTC. Same protocol as the Unitree app.

**Rule:** Only one client at a time. Close the mobile app before connecting.

---

## Three ways to interact


| Type        | What it is                     | How you use it                               |
| ----------- | ------------------------------ | -------------------------------------------- |
| **Stream**  | Robot pushes data continuously | Subscribe to a topic, handle each message    |
| **Request** | You ask, robot answers once    | Send to a `*/request` topic with an `api_id` |
| **Media**   | Camera or microphone           | Separate video/audio channel (not a topic)   |


Streams are for sensors. Requests are for settings, tricks, and snapshots.

---

## What you can read (from the dog)

### Always available (idle robot)

These stream without starting anything in the app:


| Interface                         | What you get                                                                      |
| --------------------------------- | --------------------------------------------------------------------------------- |
| **Battery & body** (`LOW_STATE`)  | Voltage, current, IMU tilt, motor temps, foot pressure                            |
| **Motion** (`LF_SPORT_MOD_STATE`) | Position, speed, gait, obstacle ranges                                            |
| **Pose** (`ROBOTODOM`)            | X/Y/Z position and orientation in the map frame                                   |
| **Lidar** (`ULIDAR_ARRAY`)        | Point cloud (~15 Hz, tens of thousands of points). Turn on first via lidar switch |
| **Lidar health** (`ULIDAR_STATE`) | Scan rate, errors, dirty lens %                                                   |
| **UWB tag** (`UWB_STATE`)         | Distance and angle to handheld tag (if paired)                                    |
| **System** (`MULTIPLE_STATE`)     | Volume, brightness, obstacle-avoid on/off                                         |


### Ask once (request, not a stream)

| Interface | What you get |
|-----------|--------------|
| **Sport settings** | Body height, foot height, speed level, current state |
| **Motion mode** | e.g. `mcf`, `normal`, `ai` |
| **Obstacle avoid** | On or off |
| **VUI** | Volume and brightness levels |
| **Audio library** | List of sounds on the robot |
| **Front photo** | Single JPEG snapshot |


### Only when a feature is active


| Interface                               | When it works                            |
| --------------------------------------- | ---------------------------------------- |
| **Joystick** (`WIRELESS_CONTROLLER`)    | Someone is using the physical remote     |
| **SLAM / maps** (`GRID_MAP`, `uslam/`*) | Mapping or navigation running in the app |
| **Audio playback state**                | Something is playing on the speaker      |


---

## What you can send (to the dog)

### Movement

Topic: `rt/api/sport/request`


| Action                    | Notes                                                    |
| ------------------------- | -------------------------------------------------------- |
| **Move**                  | `{x, y, z}` — forward, sideways, turn. Body-frame speeds |
| **Stop**                  | Halt immediately                                         |
| **Stand up / down / sit** | Basic poses                                              |
| **Tricks**                | Hello, dance, flips, handstand, etc.                     |


Your Pro runs in **MCF mode** — use MCF command IDs from `SPORT_CMD_MCF` in the SDK (not the older normal-mode table).

### Safety & driving style


| Interface              | What it does                                                                                                |
| ---------------------- | ----------------------------------------------------------------------------------------------------------- |
| **Obstacle avoidance** | Turn on/off; when on, drive via simulated joystick (`WIRELESS_CONTROLLER`) and the robot filters collisions |
| **Motion mode**        | Switch between `normal`, `ai`, `mcf`, etc.                                                                  |


### Lights & sound


| Interface     | What it does                                       |
| ------------- | -------------------------------------------------- |
| **VUI**       | LED color/flash, screen brightness, speaker volume |
| **Audio hub** | Play, pause, upload, and manage sound files        |


### Sensors


| Interface        | What it does                            |
| ---------------- | --------------------------------------- |
| **Lidar switch** | Turn lidar on or off (`"on"` / `"OFF"`) |

---

## Controlling where the dog goes

Navigation and poses are **different things**. Poses/tricks are choreographed motions. Navigation means reaching a place on the map.

### Option 1 — Velocity teleop (manual drive)

Topic: `rt/api/sport/request`, command **Move** (`SPORT_CMD_MCF["Move"]` on Pro).

Send repeatedly (~20 Hz) while driving:

| Field | Meaning |
|-------|---------|
| `x` | Forward / back speed (m/s) |
| `y` | Strafe left / right (m/s) |
| `z` | Turn rate (rad/s) |

Stop with **StopMove**. Good for keyboard or gamepad control, not for “go to the kitchen.”

### Option 2 — Go to a map point (this repo)

The dashboard plans a path on the floor plan. The **backend** turns that into simple dog commands:

1. Read start pose from **`ROBOTODOM`** once.
2. For each leg: **turn** to face the next point, then **go N meters straight**.
3. Send timed **Move** commands (`SPORT_CMD_MCF`) at fixed speed — navigation logic stays on our side.

Example plan for a 2 m leg after a 90° turn:

| Step | Command | Duration |
|------|---------|----------|
| 1 | Turn left 90° @ 0.4 rad/s | ~3.9 s |
| 2 | Forward 2.0 m @ 0.25 m/s | 8.0 s |

**API:** `POST /api/live/follow-path` (waypoints). Logic in `backend/src/live/navigation.py`.

Response includes the planned `segments` list so you can see what was sent.

**Obstacle pause:** While moving, the backend watches:

- `range_obstacle` from sport state (front / left / right proximity, metres)
- Forward lidar cone (~0.45 m)

If something is too close, the dog **stops** and waits until the path is clear for ~0.3 s, then **resumes** the same segment. Poll `GET /api/live/navigation` — `status: "paused_obstacle"` while waiting. Cancel with `POST /api/live/stop-navigation`.

### Option 3 — Safe joystick drive

When obstacle avoidance is **on**:

1. Enable via `rt/api/obstacles_avoid/request` (`SWITCH_SET` → `enable: true`).
2. Publish joystick values to **`rt/wirelesscontroller`**: `lx`, `ly`, `rx`, `ry`, `keys`.

The robot filters your input to avoid collisions — same idea as the Unitree app’s safe drive.

### What to use when

| Goal | Use |
|------|-----|
| Drive around manually | **Move** (repeat) or **WIRELESS_CONTROLLER** |
| Walk to a spot on the map | **follow-path** (segment planner in backend) |
| Wave, sit, dance, flip | Built-in poses (below) — not navigation |

Navigation uses **`SPORT_CMD_MCF`** (Move / StopMove / BalanceStand). Open-loop timed segments — no obstacle-avoid or odom feedback loop.

---

## Built-in poses & tricks

All on `rt/api/sport/request`. On Go2 Pro use **`SPORT_CMD_MCF`** (see `unitree_webrtc_connect/constants.py`).

**Try them interactively:**

```bash
python unitree_webrtc_connect/examples/go2/data_channel/sportmode_mcf/sportmode_mcf.py
```

### Stances

| Command | What it does |
|---------|--------------|
| StandUp | Stand up |
| StandDown | Lie down |
| Sit / RiseSit | Sit and get back up |
| BalanceStand | Balanced standing pose |
| RecoveryStand | Recover after a fall |
| Damp | Motors passive |

### Gestures & tricks

| Command | What it does |
|---------|--------------|
| Hello | Wave |
| Stretch | Stretch |
| Heart | Heart gesture |
| Dance1 / Dance2 | Dance routines |
| Scrape | Scrape motion |
| FrontFlip / BackFlip / LeftFlip | Flips (often needs `{"data": true}`) |
| FrontJump / FrontPounce | Jump / pounce |
| HandStand | Handstand on/off (`{"data": true/false}`) |

### Gait & walk modes

Toggle with `{"data": true}` or `{"data": false}`:

| Command | What it does |
|---------|--------------|
| FreeWalk | Free walk mode |
| ClassicWalk | Classic walk |
| TrotRun | Trot run |
| EconomicGait | Power-saving gait |
| StaticWalk | Static walk |
| CrossStep | Cross step |
| LeadFollow | Follow mode |

### Custom posing

| Command | How |
|---------|-----|
| **Pose** | `{"data": true}` — enter pose mode (like the app pose editor). Exit with **StopMove**. |
| **Euler** | `{"x": roll, "y": pitch, "z": yaw}` — tilt the body |

### Example — Hello gesture

```python
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD_MCF

await conn.datachannel.pub_sub.publish_request_new(
    RTC_TOPIC["SPORT_MOD"],
    {"api_id": SPORT_CMD_MCF["Hello"]},
)
```

### Example — Move forward (teleop)

Move is fire-and-forget (no reply). Send often while holding a key:

```python
await conn.datachannel.pub_sub.publish(
    RTC_TOPIC["SPORT_MOD"],
    {
        "api_id": SPORT_CMD_MCF["Move"],
        "parameter": {"x": 0.3, "y": 0, "z": 0},
    },
)
```

---

## Media


| Channel   | Direction | Notes                                     |
| --------- | --------- | ----------------------------------------- |
| **Video** | Dog → you | Front camera, ~720p, live H.264 stream    |
| **Photo** | Dog → you | One-shot JPEG via request                 |
| **Audio** | Both ways | Hear robot mic; play sounds via audio hub |


---

## Topic cheat sheet

Streams (subscribe):

```
rt/lf/lowstate              battery, IMU, motors
rt/lf/sportmodestate        motion state  ← use this on Pro
rt/utlidar/robot_pose       position
rt/utlidar/voxel_map_compressed   lidar points
rt/utlidar/lidar_state      lidar health
rt/multiplestate            volume, brightness, switches
rt/uwbstate                 UWB tag
rt/wirelesscontroller       joystick (when active)
```

Requests (send + wait for reply):

```
rt/api/sport/request        move, tricks, queries
rt/api/motion_switcher/request   mode
rt/api/obstacles_avoid/request   collision avoidance
rt/api/vui/request            LED, volume, brightness
rt/api/audiohub/request       sounds
rt/api/videohub/request       photo
rt/utlidar/switch             lidar on/off (publish, no reply)
```

Full names and IDs: `unitree_webrtc_connect/constants.py` (`RTC_TOPIC`, `SPORT_CMD`, `SPORT_CMD_MCF`).

---

## Complete topic catalog

Every channel defined in `RTC_TOPIC` (`unitree_webrtc_connect/constants.py`). This dashboard **subscribes to all stream topics** on connect (request/publish-only topics are excluded — see `SKIP_SUBSCRIBE` in `backend/src/recording/util.py`).

**Legend**

| Column | Meaning |
| ------ | ------- |
| **Dir** | `←` robot pushes to you (subscribe), `→` you send to robot (request/publish), `↔` both |
| **Dashboard** | How this repo uses the stream |

### Core sensors & state (always useful)

| Label | Topic | Dir | Rate / notes | Payload (key fields) | Dashboard |
| ----- | ----- | --- | ------------ | -------------------- | --------- |
| `LOW_STATE` | `rt/lf/lowstate` | ← | ~50 Hz | `power_v`, `bms_state` (soc, current), `imu_state`, `motor_state[]` (angle, temp), `foot_force[4]` | Battery panel, recorded |
| `LF_SPORT_MOD_STATE` | `rt/lf/sportmodestate` | ← | ~50 Hz | `mode`, `gait_type`, `body_height`, `yaw_speed`, `range_obstacle[4]` (front/left/back/right m), `imu_state` (rpy, gyro, accel) | **Primary motion state on Go2 Pro.** Pose yaw fusion, obstacle pause, recorded |
| `ROBOTODOM` | `rt/utlidar/robot_pose` | ← | ~15 Hz | `pose.position` (x,y,z), `pose.orientation` (quaternion) | Map pose, floor-plan path, navigation, recorded |
| `ULIDAR_ARRAY` | `rt/utlidar/voxel_map_compressed` | ← | ~15 Hz | Compressed voxel map → decoded to N×3 `float32` points (x,y,z), plus `origin`, `resolution`, `frame_id`, `stamp` | 3D lidar view, floor-plan builder, recorded as `.npz` |
| `ULIDAR` | `rt/utlidar/voxel_map` | ← | ~15 Hz | Uncompressed voxel map (same geometry, larger messages) | Subscribed & recorded; dashboard prefers `ULIDAR_ARRAY` |
| `ULIDAR_STATE` | `rt/utlidar/lidar_state` | ← | ~1 Hz | `cloud_frequency`, `cloud_size`, `error_state`, `dirty_percentage` | Lidar health UI, recorded |
| `MULTIPLE_STATE` | `rt/multiplestate` | ← | on change | `volume`, `brightness`, `obstaclesAvoidSwitch`, `uwbSwitch` | System switches UI, recorded |
| `UWB_STATE` | `rt/uwbstate` | ← | ~10 Hz | `distance_est`, `yaw_est`, `pitch_est`, `joystick[2]`, `buttons`, `enabled_from_app` | UWB panel (if paired), recorded |

### Lidar control (publish only — not subscribed)

| Label | Topic | Dir | Notes |
| ----- | ----- | --- | ----- |
| `ULIDAR_SWITCH` | `rt/utlidar/switch` | → | Publish `"on"` or `"OFF"` (no reply). Dashboard turns lidar on at connect. |

### Legacy / alternate sport state

| Label | Topic | Dir | Notes | Dashboard |
| ----- | ----- | --- | ----- | --------- |
| `SPORT_MOD_STATE` | `rt/sportmodestate` | ← | Pre-MCF sport state topic | Subscribed & recorded; **Pro uses `LF_SPORT_MOD_STATE` instead** |

### Input devices & low-level motor

| Label | Topic | Dir | When active | Payload | Dashboard |
| ----- | ----- | --- | ----------- | ------- | --------- |
| `WIRELESS_CONTROLLER` | `rt/wirelesscontroller` | ← | Physical remote in use | `lx`, `ly`, `rx`, `ry`, `keys` | Recorded; used for safe-drive / obstacle-avoid teleop pattern |
| `LOW_CMD` | `rt/lowcmd` | ↔ | Low-level motor commands | Per-motor position/torque targets | Subscribed & recorded (not used by dashboard logic) |

### Audio & VUI

| Label | Topic | Dir | Notes | Dashboard |
| ----- | ----- | --- | ----- | --------- |
| `AUDIO_HUB_PLAY_STATE` | `rt/audiohub/player/state` | ← | `play_state`, `is_playing`, track name/ID | Audio player UI, recorded |
| `AUDIO_HUB_REQ` | `rt/api/audiohub/request` | → | Play/pause/upload/list sounds (`AUDIO_API` IDs) | RPC at session start |
| `VUI` | `rt/api/vui/request` | → | LED color, volume, brightness (`api_id` per action) | RPC at session start |

### Sport, motion & safety (requests — you send)

| Label | Topic | Dir | Notes | Dashboard |
| ----- | ----- | --- | ----- | --------- |
| `SPORT_MOD` | `rt/api/sport/request` | → | Move, tricks, poses. **Pro: use `SPORT_CMD_MCF` IDs** | Teleop, navigation, tricks |
| `MOTION_SWITCHER` | `rt/api/motion_switcher/request` | → | Switch `normal` / `ai` / `mcf` | RPC query at record start |
| `OBSTACLES_AVOID` | `rt/api/obstacles_avoid/request` | → | Enable/disable collision avoidance, API-driven move | RPC + safe-drive pattern |
| `FRONT_PHOTO_REQ` | `rt/api/videohub/request` | → | Single JPEG snapshot (`api_id: 1001`) | RPC at record start |

### Unitree app SLAM / mapping (only when app mapping is running)

| Label | Topic | Dir | Payload | Dashboard |
| ----- | ----- | --- | ------- | --------- |
| `GRID_MAP` | `rt/mapping/grid_map` | ← | 2D occupancy grid from Unitree app | Recorded only |
| `SLAM_ODOMETRY` | `rt/lio_sam_ros2/mapping/odometry` | ← | LIO-SAM mapping odometry | Recorded only |
| `SLAM_QT_COMMAND` | `rt/qt_command` | ↔ | SLAM UI commands | Recorded only |
| `SLAM_ADD_NODE` | `rt/qt_add_node` | ↔ | Graph node for SLAM | Recorded only |
| `SLAM_ADD_EDGE` | `rt/qt_add_edge` | ↔ | Graph edge for SLAM | Recorded only |
| `SLAM_QT_NOTICE` | `rt/qt_notice` | ← | SLAM status notices | Recorded only |
| `SLAM_PC_TO_IMAGE_LOCAL` | `rt/pctoimage_local` | ← | Local point cloud preview image | Recorded only |

### USLAM (Unitree lidar mapping stack — app feature)

| Label | Topic | Dir | Payload | Dashboard |
| ----- | ----- | --- | ------- | --------- |
| `LIDAR_MAPPING_CLOUD_POINT` | `rt/uslam/frontend/cloud_world_ds` | ← | Downsampled world point cloud | Recorded only |
| `LIDAR_MAPPING_ODOM` | `rt/uslam/frontend/odom` | ← | Mapping odometry | Recorded only |
| `LIDAR_MAPPING_PCD_FILE` | `rt/uslam/cloud_map` | ← | Saved map point cloud | Recorded only |
| `LIDAR_MAPPING_SERVER_LOG` | `rt/uslam/server_log` | ← | USLAM server logs | Recorded only |
| `LIDAR_LOCALIZATION_ODOM` | `rt/uslam/localization/odom` | ← | Localization pose | Recorded only |
| `LIDAR_LOCALIZATION_CLOUD_POINT` | `rt/uslam/localization/cloud_world` | ← | Localization map cloud | Recorded only |
| `LIDAR_NAVIGATION_GLOBAL_PATH` | `rt/uslam/navigation/global_path` | ← | Planned global path | Recorded only |
| `LIDAR_MAPPING_CMD` | `rt/uslam/client_command` | → | USLAM client commands | Not subscribed |

### Accessories & misc streams

| Label | Topic | Dir | Notes | Dashboard |
| ----- | ----- | --- | ----- | --------- |
| `ARM_COMMAND` | `rt/arm_Command` | → | Robotic arm commands (if equipped) | Not subscribed |
| `ARM_FEEDBACK` | `rt/arm_Feedback` | ← | Arm state feedback | Recorded only |
| `GAS_SENSOR` | `rt/gas_sensor` | ← | Gas sensor readings (if equipped) | Recorded only |
| `GAS_SENSOR_REQ` | `rt/api/gas_sensor/request` | → | Gas sensor queries | Not subscribed |
| `SERVICE_STATE` | `rt/servicestate` | ← | On-robot service health | Recorded only |
| `SELF_TEST` | `rt/selftest` | ← | Self-test diagnostics | Recorded only |
| `GPT_FEEDBACK` | `rt/gptflowfeedback` | ← | GPT / voice-assistant flow feedback | Recorded only |
| `PROGRAMMING_ACTUATOR_CMD` | `rt/programming_actuator/command` | → | Blockly / programming actuator | Not subscribed |
| `ASSISTANT_RECORDER` | `rt/api/assistant_recorder/request` | → | Voice assistant recorder API | Not subscribed |
| `BASH_REQ` | `rt/api/bashrunner/request` | → | Run shell commands on robot | Not subscribed |
| `UWB_REQ` | `rt/api/uwbswitch/request` | → | Enable/disable UWB | Not subscribed |

### Media (not WebRTC data-channel topics)

| Channel | Dir | Format | Notes | Dashboard |
| ------- | --- | ------ | ----- | --------- |
| **Video** | ← | H.264 ~720p | Front camera live stream | Live preview, saved as JPEG frames (~5 Hz) during recording |
| **Audio in** | ← | Opus/PCM | Robot microphone | Available via WebRTC audio track (not parsed in dashboard) |
| **Audio out** | → | — | Play sounds via `AUDIO_HUB_REQ` | — |

### What this dashboard ingests live

On connect (`LiveManager._connection_main`), the backend:

1. Publishes `ULIDAR_SWITCH` → `"on"`.
2. Subscribes **core topics first** (wait up to 20 s for first lidar): `ULIDAR_ARRAY`, `ROBOTODOM`, `LOW_STATE`, `ULIDAR_STATE`.
3. Subscribes all remaining stream topics.
4. Opens the **video** track.

Parsed into typed buffers for the UI:

| Buffer | Source topic | Parser |
| ------ | ------------ | ------ |
| `odom` | `ROBOTODOM` | `parse_pose()` |
| `sport` | `LF_SPORT_MOD_STATE` | `parse_sport()` |
| `battery` | `LOW_STATE` | `parse_battery_state()` |
| `ulidar_state` | `ULIDAR_STATE` | `parse_lidar_state()` |
| `uwb` | `UWB_STATE` | `parse_uwb()` |
| `multiple_state` | `MULTIPLE_STATE` | `parse_system()` |
| `audio_hub` | `AUDIO_HUB_PLAY_STATE` | `parse_audio()` |
| `lidar` | `ULIDAR_ARRAY` | `extract_lidar_points()` → `.npz` |

All other subscribed topics are still written to `topics/{LABEL}.jsonl` in session recordings.

### Parsed field reference

Key fields extracted by `backend/src/parsing/topics.py`:

**`ROBOTODOM` → pose**

- `x`, `y`, `z`, `yaw` (yaw derived from quaternion)
- `qx`, `qy`, `qz`, `qw`

**`LF_SPORT_MOD_STATE` → sport**

- `mode`, `gait_type`, `body_height`, `yaw_rate`, `error_code`
- `range_obstacle`: `[front, left, back, right]` metres (0 = clear)
- `imu`: `rpy`, `gyro`, `accel`, `temperature`

**`LOW_STATE` → battery**

- `voltage`, `soc`, `current_ma`, `temperature_c`
- `foot_force[4]`, `motors[]` with `angle` and `temperature`

**`ULIDAR_STATE`**

- `cloud_frequency`, `cloud_size`, `error_state`, `dirty_percentage`

**`ULIDAR_ARRAY` → points**

- Body/odom-oriented N×3 float array; z matches 3D panel colour scale (~−0.5 to 1.6 m useful band)
- Metadata: `frame_id` (usually `"odom"`), `origin[3]`, `resolution`, `stamp`

**`MULTIPLE_STATE`**

- `volume`, `brightness`, `obstacles_avoid`, `uwb_switch`

**`UWB_STATE`**

- `distance`, `yaw`, `pitch`, `orientation`, `joystick`, `buttons`, `enabled_from_app`

**`AUDIO_HUB_PLAY_STATE`**

- `play_state`, `is_playing`, `track`