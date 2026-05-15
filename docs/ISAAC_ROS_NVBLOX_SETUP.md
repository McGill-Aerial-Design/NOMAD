# Isaac ROS + nvblox + ZED -- Complete Setup Guide

> **Refactor note:** one-time provisioning now lives in
> `scripts/setup/provision_isaac_ros.sh`. Day-to-day lifecycle is managed by
> per-service systemd units:
> ```
> nomad start isaac_ros_container   # the container itself
> nomad start zed_wrapper           # ZED ROS2 wrapper
> nomad start ros_http_bridge       # bridge to Edge Core
> nomad start nvblox                # nvblox (opt-in)
> ```
> Wherever this guide says `./scripts/run/start_isaac_ros_auto.sh`, use the
> commands above instead.

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
      |-- ZED SDK 5.2.3
        |-- ZED ROS2 wrapper  (camera driver, VIO at 30 Hz)
                |-- nvblox             (3D reconstruction, mesh at 5.0 Hz)
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

# ZED ROS2 wrapper (v5.2.0 is compatible with ZED SDK 5.2.x)
git clone --branch v5.2.0 --depth 1 https://github.com/stereolabs/zed-ros2-wrapper.git
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
apt update
apt install -y \
  ros-humble-zed-msgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
  ros-humble-robot-localization \
  ros-humble-point-cloud-transport \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-isaac-ros-managed-nitros \
  ros-humble-isaac-ros-nitros \
  ros-humble-isaac-ros-nitros-image-type \
  ros-humble-isaac-ros-nitros-camera-info-type \
  ros-humble-isaac-ros-nitros-point-cloud-type \
  gir1.2-gstreamer-1.0 \
  gir1.2-gst-plugins-base-1.0 \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-rtsp \
  gstreamer1.0-x

# Source workspace
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

# Launch NOMAD ZED + nvblox stack
ros2 launch /workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py
```

The NOMAD launch file already uses the `zed2` camera profile for ZED 2i.

---

## Nav2 Dependencies

Nav2 (Navigation2) is required for full autonomous navigation and mission planning features. The following packages are automatically installed by both the startup script and Docker image:

### Required Nav2 Packages

| Package | Purpose |
|---------|----------|
| `ros-humble-navigation2` | Core navigation stack (costmaps, planners, controllers) |
| `ros-humble-nav2-bringup` | Bringup utilities and launch files |
| `ros-humble-nav2-msgs` | Message definitions for nav2 bridge compatibility |

### Installation Verification

To verify Nav2 packages are installed inside the container:

```bash
# Enter container
docker exec -it nomad_isaac_ros bash

# Check packages
dpkg -l | grep ros-humble-navigation2
dpkg -l | grep ros-humble-nav2

# Or check via ROS2
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -i nav2
```

Expected packages in output:
- `nav2_behavior_tree`
- `nav2_bringup`
- `nav2_costmap_2d`
- `nav2_dynamic_params`
- `nav2_lifecycle_manager`
- `nav2_map_server`
- `nav2_navfn_planner`
- `nav2_util`

### Integration with ROS-HTTP Bridge

Nav2 messages are bridged to the Edge Core API via the ROS-HTTP bridge. This enables remote mission planning from the Ground Station:

```python
# Edge Core will expose nav2 topics if bridge is running
ros2 topic list | grep nav2  # Inside container

# Ground Station can poll status via API
GET /api/nav/status
```

If Nav2 packages are missing or Nav2 validation fails, startup may continue in degraded mode (telemetry/VIO/mesh paths can still come up), but nav2 message bridging and autonomous navigation commands will remain unavailable until Nav2 is installed and validated.

---

## nvblox Configuration

### nvblox Configuration Profiles

A unified performance configuration is currently available:

#### Performance config: `config/nvblox_performance.yaml`

Optimized for both **Task 1 (outdoor)** and **Task 2 (indoor)** on the Orin Nano 8GB.

| Parameter | Value | Notes |
|-----------|-------|-------|
| `voxel_size` | 0.10 m | 10cm voxels tuned for Orin Nano 8GB stability |
| `map_clearing_radius_m` | 3.0 m | 3m active mapping radius to reduce GPU memory pressure |
| `projective_integrator_max_integration_distance_m` | 5.0 m | Max depth integration range |
| `integrate_depth_rate_hz` | 5.0 | Conservative depth integration rate for stability |
| `update_mesh_rate_hz` | 10.0 | Mesh updates for real-time visualization |
| `update_esdf_rate_hz` | 10.0 | ESDF update rate for obstacle field updates |
| `esdf_mode` | "3d" | 3D ESDF mode for full obstacle detection |
| `mapping_type` | "dynamic" | Dynamic mapping mode for robust operation |
| `back_projection_subsampling` | 4 | Heavy subsampling for speed |
| `publish_layer_rate_hz` | 5.0 | Layer publishing rate tuned for Orin Nano headroom |



To use this config when launching nvblox manually:

```bash
ros2 launch /workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py
```

### ZED Resolution

The ZED wrapper defaults to 360p (downscale factor 2.0). The startup script keeps this default. To switch to native 720p manually:

Edit `install/zed_wrapper/share/zed_wrapper/config/common.yaml`:
```yaml
pub_downscale_factor: 1.0  # 1.0 = native 720p, 2.0 = 360p
```

---

## Key ROS2 Topics

| Topic | Rate | Description |
|-------|------|-------------|
| `/zed/zed_node/rgb/image_rect_color` | 30 Hz | Camera image (default 360p via `pub_downscale_factor: 2.0`; optional 720p with `pub_downscale_factor: 1.0`) |
| `/zed/zed_node/depth/depth_registered` | 30 Hz | Depth map |
| `/zed/zed_node/odom` | 30 Hz | Visual-inertial odometry |
| `/nvblox_node/mesh` | ~5 Hz (publisher-limited) | 3D mesh for RViz (NOT consumed by NOMAD bridge) |
| `/nvblox_node/static_map_slice` | ~1 Hz | 2D occupancy grid |
| `/nvblox_node/color_layer_marker` | publisher-limited; forwarded by ros_http_bridge at up to 30 Hz | Colored voxel markers (primary mesh source for Mission Planner) |

---

## Data Flow to Mission Planner

Primary real-time path is the WebSocket stream (`/ws/slam`), fed by marker-based mesh updates plus VIO pose.

```
/nvblox_node/color_layer_marker (ROS2 Marker CUBE_LIST, 5.0 Hz)
    + /zed/zed_node/odom (ROS2 topic, 30 Hz)
    |
    v
ros_http_bridge.py (inside container)
    |  Converts marker data to voxel payloads (mode="voxel")
    |  POST /api/task/2/slam/mesh/update
    v
Edge Core API (host, port 8000)
    |  WebSocket /ws/slam (primary real-time pose + mesh stream)
    |  REST GET /api/task/2/slam/mesh (snapshot/fallback)
    v
Mission Planner Plugin (Windows, over Tailscale VPN)
    |
    v
SLAM3DView.cs (OpenTK / cross-platform OpenGL)
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
- Verify `esdf_mode` in the selected profile (current performance profile uses `"2d"`; set `"3d"` only when using a profile that requires it)

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
*Isaac ROS Version: 3.2 | ROS2: Humble | JetPack: 6.2 | ZED SDK: 5.2.3*
