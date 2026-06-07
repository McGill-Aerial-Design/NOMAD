# ZED Isaac Sim Integration

NOMAD integrates ZED Isaac Sim to provide a **full hardware-free simulation**
that is indistinguishable from a real Jetson + ZED2i + flight controller setup.
Edge Core and the Mission Planner see the same API and ROS2 topics whether
the source is Isaac Sim or real hardware.

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│  Dev Workstation (x86_64, NVIDIA GPU)                            │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  Isaac Sim Container (nomad_isaac_sim)                    │   │
│  │  ┌────────────────────────────────────────────────────┐  │   │
│  │  │  Isaac Sim 4.5 (headless Vulkan rendering)         │  │   │
│  │  │  ZED Sim Publisher  →  /zed/zed_node/*             │  │   │
│  │  │  ROS-HTTP Bridge    →  Edge Core :8000             │  │   │
│  │  │  Video Bridge       →  MediaMTX :8554              │  │   │
│  │  └────────────────────────────────────────────────────┘  │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  ┌──────────────────────┐   ┌───────────────────────────┐      │
│  │  Edge Core (:8000)   │   │  ArduPilot SITL (opt-in)  │      │
│  │  Same API as Jetson  │   │  MAVLink on TCP :5760     │      │
│  └──────────────────────┘   └───────────────────────────┘      │
│                                                                  │
│  ┌──────────────────────┐                                      │
│  │  MediaMTX (:8554)    │  RTSP relay                         │
│  └──────────────────────┘                                      │
└──────────────────────────────────────────────────────────────────┘

Mission Planner connects to:
  - REST API:  http://localhost:8000
  - MAVLink:   TCP localhost:5760  (with --profile sitl)
  - RTSP:      rtsp://localhost:8554/primary
  - WebSocket: ws://localhost:8000/ws/state
```

## Prerequisites

1. **NVIDIA GPU** with Vulkan support (RTX 2000+ series recommended)
2. **NVIDIA Container Toolkit** — install from
   [docs.nvidia.com/datacenter/cloud-native/container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
3. **Docker** 24+ with Compose v2
4. **Pixi** — [pixi.sh](https://pixi.sh) (for task shortcuts)

Verify GPU access:
```bash
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi
```

## Quick Start

### Option A: Docker Compose (recommended)

```bash
# Build all images
docker compose -f docker/docker-compose.sim.yml build

# Start the full simulation stack
docker compose -f docker/docker-compose.sim.yml up

# With ArduPilot SITL for MAVLink:
docker compose -f docker/docker-compose.sim.yml --profile sitl up
```

### Option B: Pixi tasks

```bash
# Activate the sim environment
pixi shell -e sim

# Build the Isaac Sim image
pixi run sim-build

# Start the stack
pixi run sim-up

# With SITL
pixi run sim-up-sitl

# View logs
pixi run sim-logs

# Stop
pixi run sim-down
```

### Option C: Manual container management

```bash
# Build the Isaac Sim image
docker build -f docker/Dockerfile.isaac_sim -t nomad-isaac-sim:latest .

# Start the container
nomad start isaac_sim

# Start the ZED sim publisher
nomad start isaac_sim_zed

# Start Edge Core with sim mode
ISAAC_SIM_MODE=true python -m edge_core.main --sim --host 0.0.0.0 --port 8000
```

## What Works in Sim

Everything that works on the real Jetson works in sim:

| Feature | Real Hardware | Isaac Sim |
|---------|:---:|:---:|
| Edge Core REST API | ✅ | ✅ |
| VIO odometry | ✅ | ✅ (simulated) |
| IMU / magnetometer | ✅ | ✅ (simulated) |
| RGB video (RTSP) | ✅ | ✅ (synthetic) |
| Depth maps | ✅ | ✅ (synthetic) |
| Object detections | ✅ | ✅ (simulated) |
| MAVLink telemetry | ✅ | ✅ (via SITL) |
| Servo control | ✅ | ⚠️ (no-op in sim) |
| nvblox mesh | ✅ | ⚠️ (no real 3D) |
| Mission Planner | ✅ | ✅ |

## ROS2 Topic Compatibility

The ZED simulation publisher emits the exact same topics and message types
as the real ZED ROS2 wrapper on the Jetson:

| Topic | Type | Rate | Source |
|-------|------|------|--------|
| `/zed/zed_node/odom` | `nav_msgs/Odometry` | 30 Hz | Sim VIO |
| `/zed/zed_node/imu/data` | `sensor_msgs/Imu` | 200 Hz | Sim IMU |
| `/zed/zed_node/imu/mag` | `sensor_msgs/MagneticField` | 100 Hz | Sim mag |
| `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/Image` | 15 Hz | Sim camera |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` | 15 Hz | Sim depth |
| `/zed/zed_node/obj_det/objects` | `zed_interfaces/ObjectsStamped` | 10 Hz | Sim OD |
| `/cmd_vel` | `geometry_msgs/Twist` | on-demand | nav2 |

Frame IDs also match exactly:
- `zed_camera_link`
- `zed_left_camera_frame`
- `zed_left_camera_optical_frame`
- `zed_depth_camera_frame`
- `zed_depth_optical_frame`

## Configuration

All sim settings are in `config/nomad.env` (copy from `config/nomad.env.example`):

```bash
# Enable sim mode (set this on dev workstations, NOT on the Jetson)
ISAAC_SIM_MODE=true
ISAAC_SIM_CONTAINER_NAME=nomad_isaac_sim
ISAAC_SIM_IMAGE_NAME=nomad-isaac-sim:latest

# Headless vs windowed
ISAAC_SIM_HEADLESS=1

# Initial drone pose
ISAAC_SIM_DRONE_START_X=0.0
ISAAC_SIM_DRONE_START_Y=0.0
ISAAC_SIM_DRONE_START_Z=1.0
ISAAC_SIM_DRONE_START_YAW=0.0

# Autostart
NOMAD_AUTOSTART_ISAAC_SIM=true
NOMAD_AUTOSTART_ISAAC_SIM_ZED=true
```

## API Endpoints

In addition to the standard Edge Core API, the sim exposes:

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/sim/status` | GET | Container + ZED publisher status |
| `/api/sim/start` | POST | Start the Isaac Sim container |
| `/api/sim/stop` | POST | Stop the container |
| `/api/sim/zed/start` | POST | Start the ZED sim publisher |
| `/api/sim/zed/stop` | POST | Stop the ZED sim publisher |
| `/api/sim/pose` | GET | Current simulated drone pose |

The standard Isaac routes (`/api/isaac/*`) are also sim-aware: when
`ISAAC_SIM_MODE=true`, they target the `nomad_isaac_sim` container
instead of the Jetson's `nomad_isaac_ros` container.

## Docker Compose Profiles

The `docker-compose.sim.yml` supports profiles:

- **Default**: Edge Core + Isaac Sim + MediaMTX (no MAVLink)
- **`sitl`**: Adds ArduPilot SITL for realistic MAVLink telemetry

```bash
# No SITL (VIO-only testing)
docker compose -f docker/docker-compose.sim.yml up

# With SITL (full stack including flight controller sim)
docker compose -f docker/docker-compose.sim.yml --profile sitl up
```

When SITL is running, connect Mission Planner to **TCP localhost:5760**.

## Building the Isaac Sim Image

The Dockerfile pulls the official NVIDIA Isaac Sim base image from
[nvcr.io](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim)
(~25 GB download, ~60 GB on disk). The build takes 30-60 minutes
depending on network speed and GPU.

```bash
docker build -f docker/Dockerfile.isaac_sim -t nomad-isaac-sim:latest .
```

The image includes:
- NVIDIA Isaac Sim 4.5
- ROS 2 Humble (full desktop + nav2)
- ZED SDK 4.2 (x86_64)
- ZED ROS2 Wrapper v4.2.1
- Isaac ROS nvblox utils
- GStreamer for video bridge
- NOMAD bridge scripts

## Troubleshooting

### GPU not detected
```bash
# Verify NVIDIA Container Toolkit
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi
```

### ZED topics not publishing
```bash
# Check container logs
docker logs nomad_isaac_sim --tail 50

# Check if the ZED sim publisher is running
docker exec nomad_isaac_sim pgrep -f zed_sim_publisher

# Manually list ROS2 topics
docker exec nomad_isaac_sim bash -c \
  "source /opt/ros/humble/setup.bash; ros2 topic list"
```

### Video bridge not streaming
```bash
# Check MediaMTX
curl -s http://localhost:9997/v3/paths/list

# Check video bridge inside the container
docker exec nomad_isaac_sim curl -s http://localhost:9200/health
```

### Permission errors with Docker socket
```bash
sudo usermod -aG docker $USER
# Log out and back in
```
