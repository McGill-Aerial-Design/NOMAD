# Isaac ROS + nvblox + ZED -- Complete Setup Guide

Everything needed to get nvblox running with a ZED camera on the Jetson Orin Nano.

---

## Architecture Overview

```
Host (Jetson Orin Nano, JetPack 6.2)
  |-- Edge Core API       (systemd / start_nomad_full.sh, port 8000)
  |-- MediaMTX             (RTSP server, port 8554)
  |-- MAVLink Router       (telemetry, port 14550)
  |-- Tailscale VPN        (systemd)
  |
  +-- Docker: nomad_isaac_ros  (isaac_ros_dev-aarch64 image)
        |-- ZED SDK 4.2
        |-- ZED ROS2 wrapper  (camera driver, VIO at 30 Hz)
        |-- nvblox             (3D reconstruction, mesh at ~7 Hz)
        |-- ROS-HTTP bridge    (relays VIO + mesh to Edge Core)
        +-- Video bridge       (managed by Edge Core VideoStreamManager API)
```

**What persists between container restarts:**
- Source code in `/workspaces/isaac_ros-dev/src/` (host volume)
- Build artifacts in `build/`, `install/`, `log/` (host volume)

**What gets wiped on container restart:**
- System packages installed with `apt install`
- ZED SDK installation
- Any container filesystem changes outside the workspace

---

## One-Time Setup (Fresh Jetson)

### 1. Create the Isaac ROS workspace

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
```

### 2. Clone required repos

```bash
# Isaac ROS common (build tools + base Dockerfiles)
git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# nvblox
git clone --branch release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git
cd isaac_ros_nvblox && git submodule update --init --depth 1 && cd ..

# ZED ROS2 wrapper (branch must match ZED SDK version)
git clone --branch humble-v4.1.4 --depth 1 https://github.com/stereolabs/zed-ros2-wrapper.git
cd zed-ros2-wrapper && git submodule update --init --recursive && cd ..
```

### 3. Build the official Isaac ROS base image

This produces the `isaac_ros_dev-aarch64` Docker image (~20-25 GB). Takes 1-2 hours on first build; subsequent builds use cache.

```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts
./build_image_layers.sh --image_key ros2_humble
```

Wait for it to finish. Verify the image exists:

```bash
docker images | grep isaac_ros_dev
# Expected: isaac_ros_dev-aarch64   latest   ...   ~22GB
```

### 4. (Optional) Build the NOMAD layer

This adds ZED SDK, GStreamer, and Python deps on top of the base image so they persist across restarts:

```bash
cd ~/NOMAD
docker compose build isaac-ros
```

If you skip this, the `start_isaac_ros_auto.sh` script will install them at runtime (slower startup, but works).

### 5. First launch

```bash
cd ~/NOMAD
./scripts/run/start_isaac_ros_auto.sh
```

The first launch will:
- Create the container
- Install ZED SDK inside it (if not baked into the image)
- Install apt dependencies
- Build ZED wrapper + nvblox with colcon (10-30 minutes first time)
- Launch ZED + nvblox + ROS-HTTP bridge

---

## Every Time You Restart the Container

If you used `docker compose build` (step 4), ZED SDK and apt deps are baked in. Just run:

```bash
./scripts/run/start_isaac_ros_auto.sh
```

If you skipped step 4 and are using the bare `isaac_ros_dev-aarch64` image, the script will auto-install deps each time. This adds a few minutes to startup.

### Manual restart flow (for debugging)

```bash
# Enter the container
docker exec -it nomad_isaac_ros bash

# Install deps manually (only needed if container was recreated)
sudo apt update
sudo apt install -y \
  ros-humble-zed-msgs \
  ros-humble-robot-localization \
  ros-humble-point-cloud-transport \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools

# Source workspace
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

# Launch nvblox with ZED
ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2
```

Use `camera:=zed2` for ZED 2i cameras (confirmed by NVIDIA -- zed2i uses the zed2 profile).

---

## nvblox Configuration

### Performance config: `config/nvblox_performance.yaml`

Key parameters for the Orin Nano:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `voxel_size` | 0.15 m | Balance of speed vs resolution |
| `map_clearing_radius_m` | 8.0 m | Mapping radius around the drone |
| `projective_integrator_max_integration_distance_m` | 8.0 m | Max depth integration range |
| `update_mesh_rate_hz` | 2.0 | Mesh output rate |
| `update_esdf_rate_hz` | 5.0 | Distance field update rate |
| `esdf_mode` | "3d" | Full volumetric mapping |
| `mapping_type` | "static_tsdf" | Best for mesh generation |
| `back_projection_subsampling` | 4 | Heavy subsampling for speed |

To use this config when launching nvblox manually:

```bash
ros2 launch nvblox_examples_bringup zed_example.launch.py \
    camera:=zed2 \
    nvblox_params_file:=/workspaces/isaac_ros-dev/config/nvblox_performance.yaml
```

### ZED Resolution

The ZED wrapper defaults to 360p (downscale factor 2.0). The startup script patches this to native 720p automatically. To change manually:

Edit `install/zed_wrapper/share/zed_wrapper/config/common.yaml`:
```yaml
pub_downscale_factor: 1.0  # 1.0 = native 720p, 2.0 = 360p
```

---

## Key ROS2 Topics

| Topic | Rate | Description |
|-------|------|-------------|
| `/zed/zed_node/rgb/image_rect_color` | 30 Hz | Camera image (720p BGRA8) |
| `/zed/zed_node/depth/depth_registered` | 30 Hz | Depth map |
| `/zed/zed_node/odom` | 30 Hz | Visual-inertial odometry |
| `/nvblox_node/mesh` | ~7 Hz | 3D mesh for visualization |
| `/nvblox_node/static_map_slice` | ~1 Hz | 2D occupancy grid |
| `/nvblox_node/color_layer_marker` | ~7 Hz | Colored voxel markers |

---

## Data Flow to Mission Planner

```
nvblox_node/mesh (ROS2 topic, ~7 Hz)
    |
    v
ros_http_bridge.py (inside container)
    |  Mesh updates rate-limited to ~2 Hz
    |  VIO pose forwarded at up to 30 Hz
    |  POST /api/task/2/slam/mesh/update
    v
Edge Core API (host, port 8000)
    |  GET /api/task/2/slam/mesh
    v
Mission Planner Plugin (Windows, over Tailscale VPN)
    |
    v
SLAM3DView.cs (Helix Toolkit WPF, 10 Hz polling)
```

---

## Verification Commands

```bash
# Check container is running
docker ps | grep nomad_isaac_ros

# Enter container
docker exec -it nomad_isaac_ros bash

# Inside container:
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

# List all packages
ros2 pkg list | grep -E 'zed|nvblox'

# Check topics
ros2 topic list | grep -E 'zed|nvblox'

# View TF tree
ros2 run tf2_tools view_frames

# Check ZED camera detection
/usr/local/zed/tools/ZED_Explorer  # Ctrl+C to exit
```

---

## Common Issues

### Container has no ZED SDK after restart
Expected. The ZED SDK lives in the container filesystem (not the mounted workspace). Re-install it or use the NOMAD Docker image (`docker compose build`).

### `ros2 pkg list` doesn't show zed_wrapper
Source the workspace: `source /workspaces/isaac_ros-dev/install/setup.bash`

### Library loading errors (libpoint_cloud_transport, librobot_localization, etc.)
Install the apt deps: see "Every Time You Restart" above.

### Transform lookup errors for `zed_camera_link`
- Verify the ZED node is running (check launch output for errors)
- Verify TF2 packages are installed
- Check camera is connected: `lsusb | grep -i stereo`

### nvblox not producing mesh
- Verify both depth and odometry topics are being published
- Check the nvblox node logs for errors
- Ensure `esdf_mode` is set to `"3d"` in the params file

---

## Cleanup / Disk Recovery

```bash
# Remove intermediate build images
docker rmi aarch64-image:latest 2>/dev/null || true

# Prune build cache
docker builder prune

# Remove dangling images
docker image prune
```

---

## File Reference

| File | Purpose |
|------|---------|
| `docker-compose.yml` | Container definition (optional, convenience) |
| `infra/docker/Dockerfile.isaac_ros_full` | NOMAD layer on top of official base |
| `infra/docker/ros_entrypoint.sh` | Container entrypoint (sources ROS2) |
| `scripts/run/start_isaac_ros_auto.sh` | Automated startup (recommended) |
| `scripts/run/start_nomad_full.sh` | Full system startup (includes Isaac ROS) |
| `config/nvblox_performance.yaml` | nvblox tuning for Orin Nano |

---

*Last Updated: February 2026*
*Isaac ROS Version: 3.2 | ROS2: Humble | JetPack: 6.2 | ZED SDK: 4.2*
