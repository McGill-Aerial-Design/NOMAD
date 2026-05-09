# Jetson-Centric Navigation Architecture

This document describes the **Jetson-centric navigation architecture** for NOMAD **Task 2 (Indoor)**. In this configuration, the Jetson handles all autonomous navigation while ArduPilot operates purely as a low-level flight controller in GUIDED mode.

**Note:** Task 1 (Outdoor) uses traditional RC pilot control via ELRS directly to ArduPilot. The Jetson is mounted for video streaming only during Task 1.

## Architecture Overview

**Task 2 Only** - Jetson-centric navigation with two command sources:
- **Primary:** Isaac ROS Nav2/Nvblox generates autonomous velocity commands
- **Backup:** Human operator can send WASD velocity commands over LTE for intervention

```
                              JETSON ORIN NANO
    +-----------------------------------------------------------------+
    |                                                                 |
    |  +-------------------+     +---------------------------+        |
    |  |   Isaac ROS       |     |      Edge Core            |        |
    |  |   (Docker)        |     |      (Python 3.13)        |        |
    |  |                   |     |                           |        |
    |  |  +-----------+    |     |  +-------------------+    |        |
    |  |  | ZED VSLAM |----+---->|  | VIO Pipeline      |    |        |
    |  |  +-----------+    |     |  +-------------------+    |        |
    |  |        |          |     |           |              |        |
    |  |        v          |     |           v              |        |
    |  |  +-----------+    |     |  +-------------------+    |        |
    |  |  | Nvblox    |    |     |  | NavController     |    |        |
    |  |  | (3D Map)  |    |     |  +-------------------+    |        |
    |  |  +-----------+    |     |           |              |        |
    |  |        |          |     |           v              |        |
    |  |        v          |     |  +-------------------+    |        |
    |  |  +-----------+    |     |  | MavlinkService    |    |        |
    |  |  | Nav2      |----+---->|  +-------------------+    |        |
    |  |  | Planner   |    |     |           |              |        |
    |  |  +-----------+    |     |           |              |        |
    |  |        |          |     |           |              |        |
    |  |        v          |     |           |              |        |
    |  |    /cmd_vel       |     |           |              |        |
    |  |        |          |     |           |              |        |
    |  |        v          |     |           |              |        |
    |  | +---------------+ |     |           |              |        |
    |  | |ros_http_bridge|-+---->+           |              |        |
    |  | +---------------+ |     |           |              |        |
    |  +-------------------+     +-----------|---------------+        |
    |                                        | MAVLink UDP            |
    +-----------------------------------------------------------------+
                                             |
                          +------------------v------------------+
                          |           mavlink-router            |
                          +------------------+------------------+
                                             |
                  +--------------------------+--------------------------+
                  |                          |                          |
                  v                          v                          v
         +----------------+        +----------------+        +----------------+
         |  Cube Orange   |        |   LTE/4G       |        |   ELRS         |
         |  (GUIDED mode) |        |   (Tailscale)  |        |   (Backup)     |
         +----------------+        +----------------+        +----------------+
                 |                         |                         |
                 v                         v                         v
         Motor Control            Mission Planner             RC Control
```

## Component Responsibilities

### Isaac ROS (Docker Container)
| Component | Responsibility |
|-----------|----------------|
| ZED VSLAM | Visual-Inertial Odometry for position estimation |
| Nvblox | 3D occupancy mapping for obstacle detection |
| Nav2 | Path planning and velocity command generation |
| ros_http_bridge | Bridges ROS topics to Edge Core HTTP API |

### Edge Core (Host Python Service)
| Component | Responsibility |
|-----------|----------------|
| VIO Pipeline | Transforms VIO to NED, sends VISION_POSITION_ESTIMATE |
| NavController | Validates and forwards velocity commands to MAVLink |
| MavlinkService | MAVLink protocol handling, sends SET_POSITION_TARGET_LOCAL_NED |
| API | HTTP endpoints for status, control, and ROS bridge |

### ArduPilot (Flight Controller)
| Function | Mode |
|----------|------|
| Attitude Control | Always active |
| Motor Mixing | Always active |
| Altitude Hold | Via barometer + VIO |
| Position Control | GUIDED mode with external velocity commands |
| Failsafe | Triggered by loss of commands or VIO failure |

## Data Flow (Task 2 Only)

### Position Feedback (30 Hz)
```
ZED Camera -> ZED ROS driver -> /zed/zed_node/odom
    -> ros_http_bridge -> POST /api/vio/update
    -> VIO Pipeline -> VISION_POSITION_ESTIMATE MAVLink
    -> ArduPilot EKF3
```

### Primary: Autonomous Navigation (10-20 Hz)
```
Nvblox Costmap -> Nav2 Planner -> /cmd_vel (Twist)
    -> ros_http_bridge -> POST /api/nav/velocity
    -> NavController -> SET_POSITION_TARGET_LOCAL_NED MAVLink
    -> ArduPilot GUIDED mode -> Motor Control
```

### Backup: WASD Human Intervention (10-20 Hz)
```
Mission Planner WASD Keys -> WASDControl.cs
    -> POST /api/nav/velocity (HTTP over LTE/Tailscale)
    -> NavController -> SET_POSITION_TARGET_LOCAL_NED MAVLink
    -> ArduPilot GUIDED mode -> Motor Control
```

**Key Insight:** WASD controls use the same `/api/nav/velocity` endpoint as Nav2, allowing seamless human takeover during Task 2 autonomous flight if intervention is needed.

## API Endpoints

### Navigation Control
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/nav/status` | GET | Navigation controller status |
| `/api/nav/velocity` | POST | Send velocity command (from ROS) |
| `/api/nav/position` | POST | Send position target |
| `/api/nav/stop` | POST | Emergency stop (zero velocity) |
| `/api/nav/enable_guided` | POST | Request GUIDED mode |

### VIO Feedback
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/vio/update` | POST | Receive VIO pose from ROS bridge |
| `/api/vio/status` | GET | VIO pipeline health |
| `/api/vio/pose` | GET | Current position/orientation |
| `/api/vio/trajectory` | GET | Path history |
| `/api/vio/reset_origin` | POST | Reset tracking origin |
| `/api/vio/calibration` | GET | ZED calibration state |
| `/api/vio/area/save` | POST | Save VIO relocalization area map |
| `/api/vio/area/load` | POST | Load VIO relocalization area map |

## ArduPilot Configuration

### Task 1 (Outdoor - RC Pilot Control)
Task 1 uses standard ArduPilot configuration with RC pilot control. No special GUIDED mode settings needed.
```
# EKF3 Source Configuration (Outdoor - GPS)
EK3_SRC1_POSXY = 3      # GPS
EK3_SRC1_VELXY = 3      # GPS
EK3_SRC1_POSZ = 1       # Baro
EK3_SRC1_YAW = 1        # Compass
```

### Task 2 (Indoor - Jetson Navigation)
```
# EKF3 Source Configuration (Indoor - VIO)
EK3_SRC1_POSXY = 6      # ExternalNav
EK3_SRC1_VELXY = 6      # ExternalNav  
EK3_SRC1_POSZ = 1       # Baro (or 6 for ExternalNav)
EK3_SRC1_YAW = 6        # ExternalNav (no magnetometer indoors)

# GUIDED Mode Settings
GUID_OPTIONS = 0        # Default GUIDED behavior
GUID_TIMEOUT = 3        # Timeout before failsafe (seconds)

# Failsafe Configuration
FS_GCS_ENABLE = 1       # Enable GCS failsafe
FS_GCS_TIMEOUT = 5      # GCS timeout (seconds)

# Arming Checks (disable GPS for indoor)
ARMING_CHECK = -33      # Skip GPS check (-33 disables GPS)
```

### Flight Mode Usage
| Mode | Usage |
|------|-------|
| STABILIZE | Manual control, pre-flight |
| ALT_HOLD | VIO failure failsafe |
| GUIDED | Autonomous navigation (Jetson control) |
| LAND | Emergency landing |

## Safety Architecture

### Command Timeout
- NavController stops vehicle if no commands received for 0.5 seconds
- Prevents runaway if ROS bridge connection lost

### VIO Health Monitoring
- NavController validates VIO confidence before accepting commands
- Minimum confidence threshold: 30%
- If VIO fails for >3 seconds: trigger ALT_HOLD failsafe

### Velocity Limits
- Horizontal: 2.0 m/s max
- Vertical: 1.0 m/s max
- Yaw rate: 1.0 rad/s max

### Graceful Degradation
```
VIO Healthy     -> Full autonomous navigation (GUIDED + Nav2)
VIO Degraded    -> Warn pilot, continue with caution
VIO Failed      -> ALT_HOLD mode, manual takeover required
```

## ROS2 Topics

### Inputs (Subscribed by ros_http_bridge)
| Topic | Type | Source | Rate |
|-------|------|--------|------|
| `/zed/zed_node/odom` | nav_msgs/Odometry | ZED ROS driver | 30 Hz |
| `/zed/zed_node/imu/data` | sensor_msgs/Imu | ZED ROS driver | ~200 Hz |
| `/zed/zed_node/imu/mag` | sensor_msgs/MagneticField | ZED ROS driver | ~50 Hz |
| `/nvblox_node/color_layer_marker` | visualization_msgs/Marker | nvblox | publisher-limited |
| `/cmd_vel` | geometry_msgs/Twist | Nav2 | 10-20 Hz |

### Other nvblox Topics (not consumed by bridge, useful in RViz)
| Topic | Type | Description |
|-------|------|-------------|
| `/nvblox_node/mesh` | nvblox_msgs/Mesh3D | 3D occupancy mesh (RViz only) |
| `/nvblox_node/static_map_slice` | nvblox_msgs/DistanceMapSlice | 2D slice for viz |

## Starting the System

### 1. Start Edge Core
```bash
cd ~/NOMAD
python -m edge_core.main
```

### 2. Start Isaac ROS Container
```bash
./scripts/run/start_isaac_ros_auto.sh start
```

### 3. Launch ROS Nodes (inside container)
The canonical NOMAD stack (used for Mission Planner parity with RViz) launches
`nomad_zed_nvblox.launch.py`, not the upstream `zed_example.launch.py`. The
`start_isaac_ros_auto.sh` script does this automatically:

```bash
# Canonical: launches nomad_zed_nvblox + ros_http_bridge at 30 Hz
./scripts/run/start_isaac_ros_auto.sh start
```

Manual launch inside the container (canonical):
```bash
ros2 launch nomad_zed_nvblox.launch.py enable_nav2:=true
python3 /workspaces/isaac_ros-dev/edge_core/ros_http_bridge.py \
    --host localhost --port 8000 --rate 30 \
    --vio-topic /zed/zed_node/odom \
    --mesh-topic /nvblox_node/color_layer_marker \
    --high-rate-transport http
```

> **Debug only:** `ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2`
> launches the upstream nvblox example without the NOMAD bridge. Mission Planner
> will show nothing on that path and any parity measurements taken against it
> are invalid. See `docs/NVBLOX_VISUALIZATION.md` for the canonical/debug split.

### 4. Enable Navigation
```bash
# Via API
curl -X POST http://localhost:8000/api/nav/enable_guided

# Or from Mission Planner - set GUIDED mode
```

## Differences from Previous Architecture

### Before (ArduPilot-Centric)
- ArduPilot handled waypoint navigation internally
- VIO fed position to EKF, AP planned paths
- Indoor navigation relied on AP's AUTO/LOITER modes

### After (Jetson-Centric)
- Jetson (Nav2) handles all path planning
- ArduPilot only executes velocity commands in GUIDED mode
- Full obstacle avoidance via Nvblox costmap
- ROS2 navigation stack (Nav2) with behavior trees

## Troubleshooting

### Navigation Not Working
1. Check VIO status: `curl http://localhost:8000/api/vio/status`
2. Check Nav status: `curl http://localhost:8000/api/nav/status`
3. Verify GUIDED mode: Check Mission Planner or `/api/nav/status`
4. Check ros_http_bridge: `docker exec nomad_isaac_ros cat /tmp/ros_bridge.log`

### Commands Not Reaching ArduPilot
1. Verify mavlink-router: `pgrep -f mavlink-routerd`
2. Check MAVLink connection: `curl http://localhost:8000/health`
3. Ensure vehicle is armed and in GUIDED mode

### VIO Failing
1. Check VIO status: `curl http://localhost:8000/api/vio/status`
2. Check ZED calibration: `curl http://localhost:8000/api/vio/calibration`
3. Verify tracking: Environment needs sufficient visual features
4. Reset origin: `curl -X POST http://localhost:8000/api/vio/reset_origin`
