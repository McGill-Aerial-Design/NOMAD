# Simulation Environments

NOMAD provides hardware-free simulation tiers for testing the ROS2 perception
path — the same bridge code (`edge_core/ros_http_bridge`) that runs on the Jetson
in production, exercised without a Jetson, ZED camera, or GPU.

## Image comparison

| Image | What runs | GPU required | Runs in CI |
|-------|-----------|:------------:|:----------:|
| `Dockerfile.dev` | Edge Core API + ArduPilot SITL (no ROS2) | No | Yes |
| `Dockerfile.sim-ros` | ROS2 Humble + real bridge node + synthetic ZED/nvblox topics | No | Yes |
| `Dockerfile.sim-gazebo` | ArduPilot SITL + Gazebo Harmonic GUI | No (GPU optional) | No |
| `Dockerfile.sim-isaac` | Full Isaac ROS + ZED SDK 5.2.3 + nvblox (x86_64) | Yes (NVIDIA) | No |
| `Dockerfile.jetson` | Production image for the Jetson Orin Nano (aarch64) | Yes (Jetson GPU) | No |

### Why three simulation tiers?

JetPack 6.2 is an aarch64/L4T stack that cannot run on x86 hosts.  The ZED SDK
and Isaac ROS perception modules (nvblox, cuVSLAM) need an NVIDIA GPU. Rather
than attempting a 1:1 replica of the device environment in CI, NOMAD mirrors it
in three tiers:

- **Tier 1 (CPU)** — `Dockerfile.sim-ros` — runs the real ROS2 bridge against
  synthetic sensor data.  No GPU, no ZED camera, no JetPack.  Runs on every PR.
- **Tier 3 (Gazebo)** — `Dockerfile.sim-gazebo` — runs ArduPilot SITL with
  Gazebo Harmonic, publishing real physics-rendered topics into the bridge.
  No GPU required; the GUI uses software GL and is exposed through noVNC. The
  ZED SDK is bypassed.
- **Tier 2 (GPU)** — `Dockerfile.sim-isaac` — installs the full Isaac ROS stack,
  ZED SDK, and nvblox on an x86_64 machine with a discrete NVIDIA GPU.  Mirrors
  the Jetson's software stack as closely as x86 allows.

## Tier 1 — CPU quickstart

### Prerequisites

- [Docker](https://docs.docker.com/engine/install/)
- [Pixi](https://pixi.sh) (for the `pixi run` wrappers — optional, you can use
  raw `docker` commands instead)

### Build the image (once)

```bash
pixi run sim-ros-build
# equivalent: docker build -f docker/Dockerfile.sim-ros -t nomad-sim-ros:latest .
```

### Bring up the ROS simulation stack

```bash
pixi run sim-ros-up
```

This starts Edge Core and the ROS-HTTP bridge against the synthetic ZED publisher
on a shared compose network.  Once up, verify the bridge is driving data into
Edge Core:

```bash
# VIO tracking confidence (expect > 0.5):
curl http://localhost:8000/api/vio/status

# Trajectory grows as the synthetic odom publisher advances:
curl http://localhost:8000/api/vio/trajectory
```

### Other compose commands

```bash
pixi run sim-ros-logs   # tail logs from all ros-profile services
pixi run sim-ros-down   # stop and remove containers
```

### Run the integration test suite

The integration test (`tests/ros/test_ros_bridge_integration.py`) is
self-contained: it spins up an in-process HTTP stub for Edge Core, starts the
real `ROSHTTPBridge` node, and runs the synthetic publisher inside the same
`rclpy` context — no compose stack, no network, no GPU.

```bash
pixi run test-ros-integration
# equivalent:
# docker run --rm -v $PWD/tests:/opt/nomad/tests nomad-sim-ros:latest \
#     python3 -m pytest tests/ros/ -v
```

The `tests/` directory is volume-mounted at runtime so the test suite always
reflects your current working tree without an image rebuild.

## Synthetic publisher topic map

`tools/sim/zed_sim_publisher.py` publishes the following topics, which match the
bridge's default subscriber configuration:

| ROS2 message type | Topic | Bridge endpoint driven | Publisher flag |
|---|---|---|---|
| `nav_msgs/Odometry` | `/zed/zed_node/odom` | `POST /api/vio/update` | always on |
| `sensor_msgs/Imu` | `/zed/zed_node/imu/data` | `POST /api/vio/update` | always on |
| `sensor_msgs/MagneticField` | `/zed/zed_node/imu/mag` | `POST /api/vio/update` | always on |
| `geometry_msgs/Twist` | `/cmd_vel` | (nav2 stand-in, logged) | `--no-cmd-vel` to disable |
| `visualization_msgs/Marker` | `/nvblox_node/color_layer_marker` | `POST /api/slam/mesh/update` | `--no-mesh` |
| `std_msgs/Float32` | `/nomad/servo/nozzle_angle` | `POST /api/servo/camera/tilt` | `--no-servo` to disable |
| `sensor_msgs/Image` | `/zed/zed_node/rgb/color/rect/image` | (raw image passthrough) | `--enable-image` to enable |

### Publisher CLI flags

Run the publisher standalone inside the image:

```bash
docker run --rm nomad-sim-ros:latest \
    python3 -m tools.sim.zed_sim_publisher \
    --rate 10          # publish rate in Hz (default: 30)
    --no-cmd-vel       # disable /cmd_vel
    --no-mesh          # disable nvblox Marker
    --no-servo         # disable nozzle-angle Float32
    --enable-image     # enable synthetic RGB Image (requires numpy)
```

### Bridge → Edge Core HTTP endpoints

| Bridge subscription | Edge Core endpoint |
|---|---|
| `/zed/zed_node/odom` + `/zed/zed_node/imu/data` + `/zed/zed_node/imu/mag` | `POST /api/vio/update` |
| `/nvblox_node/color_layer_marker` | `POST /api/slam/mesh/update` |
| `/nomad/servo/nozzle_angle` | `POST /api/servo/camera/tilt` |
| — | `GET /api/servo/camera/tilt` (read-back) |

## Tier 3 — ArduPilot SITL + Gazebo Harmonic ZED-2i

### Why Gazebo instead of synthetic data?

The Tier-1 synthetic publisher produces mathematically-correct ROS topics but no
real 3D scene. Tier 3 adds an ArduPilot SITL + Gazebo Harmonic physics
simulation with a ZED-2i-tuned gimbal camera so the bridge is driven by rendered
camera frames and physics-integrated odometry — closer to real sensor behaviour,
but still without a GPU.

**Why is the ZED SDK bypassed?**
The ZED SDK can only ingest frames from a physical ZED device, an `.svo`
recording, a network stream, or Isaac Sim. It cannot consume Gazebo-rendered
frames. Tier 3 therefore simulates the ZED ROS contract, not the ZED SDK
runtime: the Gazebo camera, IMU, magnetometer, and odometry publisher are
bridged to the same topic names the ZED ROS2 wrapper would use, so
`edge_core.ros_http_bridge` runs unchanged. Use Tier 2 when the test must run the
ZED SDK, ZED wrapper, Isaac ROS, or nvblox/cuVSLAM GPU stack.

### Topic map (Gazebo topic → ZED topic → bridge)

| Gazebo source | ROS2 message type | Topic | Bridge action |
|---|---|---|---|
| Iris odometry publisher | `nav_msgs/Odometry` | `/zed/zed_node/odom` | `POST /api/vio/update` |
| Iris IMU sensor | `sensor_msgs/Imu` | `/zed/zed_node/imu/data` | `POST /api/vio/update` |
| Iris magnetometer sensor | `sensor_msgs/MagneticField` | `/zed/zed_node/imu/mag` | `POST /api/vio/update` |
| ZED **left** camera (360p) | `sensor_msgs/Image` | `/zed/zed_node/rgb/color/rect/image` (+ `/left/image_rect_color`) | video bridge passthrough |
| ZED **right** camera (360p) | `sensor_msgs/Image` | `/zed/zed_node/right/image_rect_color` | stereo pair |
| Camera info (left/right) | `sensor_msgs/CameraInfo` | `/zed/zed_node/rgb/camera_info`, `/zed/zed_node/right/camera_info` | calibration metadata |
| gz depth camera (ground truth) | `sensor_msgs/Image` (`32FC1`) | `/zed/zed_node/depth/depth_registered` | depth |
| gz depth camera (ground truth) | `sensor_msgs/PointCloud2` | `/zed/zed_node/point_cloud/cloud_registered` | point cloud |

The ZED 2i is a **stereo depth camera**, so the model carries two 360p cameras at
the 120 mm baseline mounted on a single **pitch servo** (driven by ArduPilot
SERVO14 — it looks up/down; the upstream 3-axis gimbal is removed). ArduPilot SITL
is the flight controller in this tier. The ROS bridge listens for nav2-style
`/cmd_vel` and sends GUIDED velocity setpoints over its dedicated `pymavlink` link.
MAVProxy routes SITL MAVLink telemetry to Edge Core and to the ROS bridge container.

### Depth

The ZED SDK cannot consume Gazebo frames, so depth is produced by a **gz depth
camera** co-located with the left lens (registered to the rgb/left frame, like the
real ZED's `depth_registered`). It publishes a rendered ground-truth depth image on
`/zed/zed_node/depth/depth_registered` and a point cloud on
`/zed/zed_node/point_cloud/cloud_registered` — no GPU required, instant and
noise-free, ideal for developing depth-consuming code. The depth range follows the
ZED 2i (`NOMAD_GAZEBO_DEPTH_NEAR_M`/`FAR_M`, default 0.3–20 m).

### Prerequisites

- [Docker](https://docs.docker.com/engine/install/)
- [Pixi](https://pixi.sh) (optional — for the `pixi run` wrappers)
- No NVIDIA GPU required (software GL used by default)

### Build the image (once)

```bash
pixi run sim-gazebo-build
# equivalent:
# docker build -f docker/Dockerfile.sim-gazebo -t nomad-sim-gazebo:latest .
```

### Bring up the Gazebo simulation stack

```bash
pixi run sim-gazebo-up
```

This starts `edge_core`, `ros_bridge_gazebo`, and `gazebo` on a shared compose
network. Gazebo runs the runway world, the Iris is flown by ArduPilot SITL, and
the generated ZED-2i gimbal publishes rendered images, IMU, magnetometer, and
Gazebo odometry under ZED-compatible topic names. The profile also starts
MediaMTX and `video_bridge_gazebo`, so the Mission Planner plugin's video bridge
button adopts the sim bridge instead of looking for the Isaac ROS container.

Mission Planner has two ways to connect to the Gazebo vehicle:

- NOMAD plugin router: the Gazebo MAVProxy stream is sent to the plugin's LTE
  UDP input on host port `14560`; Mission Planner then uses the plugin's merged
  local UDP endpoint, normally `127.0.0.1:14600`.
- Direct fallback: connect Mission Planner to `TCP 127.0.0.1:5764`. This maps to
  the Gazebo SITL instance's ArduPilot port and avoids the older dev SITL bridge
  on `127.0.0.1:5762`.

Open the Gazebo GUI in a browser:

```bash
pixi run sim-gazebo-open
# or browse to http://localhost:6080/vnc.html
```

### Verify the bridge is receiving Gazebo data

```bash
# VIO tracking confidence:
curl http://localhost:8000/api/vio/status

# Trajectory grows as Gazebo odometry arrives:
curl http://localhost:8000/api/vio/trajectory
```

### Verify the video bridge

```bash
curl -H "X-API-Key: nomad-dev-key" -X POST http://localhost:8000/api/video/bridges/start
curl -H "X-API-Key: nomad-dev-key" http://localhost:8000/api/video/bridges
```

Mission Planner and VLC should use:

```text
rtsp://localhost:8554/stream
```

Inside compose, Edge Core reaches the bridge at `video_bridge_gazebo:9200`; the
RTSP publisher uses `rtsp://mediamtx:8554/stream`.

### Other compose commands

```bash
pixi run sim-gazebo-logs   # tail logs from all gazebo-profile services
pixi run sim-gazebo-down   # stop and remove containers
pixi run sim-gazebo-up-headless  # run gz with --headless-rendering
pixi run sim-gazebo-up-fast      # headless, lower camera load for RTF checks
```

### Visible GUI and headless mode

The default `pixi run sim-gazebo-up` command runs Gazebo with its GUI on a
virtual X display inside the container. The container exposes noVNC on
`http://localhost:6080/vnc.html`, so Windows and Docker Desktop hosts do not need
VcXsrv, WSLg socket mounts, or other host display plumbing.

For camera-only or CI-style runs, use:

```bash
pixi run sim-gazebo-up-headless
```

That sets `NOMAD_GAZEBO_HEADLESS=1`, and the launch starts `gz sim` with
`--headless-rendering`.

For real-time-factor tuning, use:

```bash
pixi run sim-gazebo-up-fast
```

That uses headless rendering, a `160x90` camera at `4` Hz, and a `2` ms physics
step. The Mission Planner RTSP stream and `/api/video/topics` still work, but the
Gazebo GUI is not started. On CPU-only Windows/Docker hosts, this is the mode to
use when the goal is `1.0` RTF. The visible noVNC mode is for inspecting the
vehicle and scene; it spends substantial CPU in the Gazebo GUI and software
renderer.

### Gazebo performance tuning

The integrated sim uses the upstream ArduPilot runway world, not a local
heightmap world. There is no project-local collision heightmap to downsample.
The launch instead writes an optimized temporary copy of the world and generated
Iris/ZED model:

- camera rendering defaults to `640x360` at `10` Hz, and the RTSP bridge inherits
  the same dimensions and frame rate
- `max_step_size=0.001` and `real_time_update_rate=1000` target real-time speed
- Gazebo log verbosity defaults to `2` instead of debug-level `4`
- ArduPilot SITL defaults to `SIM_RATE_HZ=400` and wall-clock mode in this Docker
  profile to avoid synthetic-clock resets when the visible GUI falls behind
- Gazebo-Classic/ODE contact tuning is opt-in because Gazebo Harmonic normally
  uses a different physics path

Override these with `NOMAD_GAZEBO_CAMERA_WIDTH`,
`NOMAD_GAZEBO_CAMERA_HEIGHT`, `NOMAD_GAZEBO_CAMERA_FPS`,
`NOMAD_GAZEBO_VERBOSITY`, `NOMAD_GAZEBO_MAX_STEP_SIZE`,
`NOMAD_GAZEBO_REAL_TIME_UPDATE_RATE`, `NOMAD_SITL_RATE_HZ`,
`NOMAD_SITL_SPEEDUP`, and `NOMAD_SITL_SYNTHETIC_CLOCK`.

Set `NOMAD_GAZEBO_APPLY_CONTACT_TUNING=1` only when you are debugging
Gazebo-Classic/ODE-style contact instability. That enables
`NOMAD_GAZEBO_CONTACT_MAX_CORRECTING_VEL`,
`NOMAD_GAZEBO_COLLISION_MAX_VEL`, `NOMAD_GAZEBO_COLLISION_MIN_DEPTH`,
`NOMAD_GAZEBO_COLLISION_KP`, and `NOMAD_GAZEBO_COLLISION_KD`.

### Attaching a real GPU for faster rendering

Add the `deploy.resources.reservations.devices` NVIDIA block to the `gazebo`
service in `docker/docker-compose.dev.yml` (shown commented in that file) and
set `LIBGL_ALWAYS_SOFTWARE=0`. Requires `nvidia-container-toolkit`.

### Forward-looking: real VIO from rendered stereo

The Gazebo gimbal camera renders synthetic images to
`/zed/zed_node/rgb/color/rect/image`. These can instead be fed to
[Isaac ROS cuVSLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
on a GPU host for real visual-inertial odometry — replacing Gazebo odometry with
an estimate derived from the rendered frames. This bridges
Tier 3 (Gazebo physics) and Tier 2 (Isaac ROS GPU stack) in a single pipeline.

## Tier 2 — GPU / Isaac ROS setup

### Prerequisites

- An x86_64 machine with an NVIDIA GPU
- **Linux / WSL2 on Windows:** `nvidia-container-toolkit` installed and configured
- The Isaac ROS x86_64 dev base image built locally via
  [isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common):

```bash
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd isaac_ros_common
./scripts/build_image_layers.sh
# The resulting image will be tagged: isaac_ros_dev-x86_64
```

### Build the Tier-2 image

```bash
docker build -f docker/Dockerfile.sim-isaac \
    --build-arg BASE_IMAGE=isaac_ros_dev-x86_64 \
    -t nomad-sim-isaac:latest .
```

This downloads the ZED SDK 5.2.3 (kept in lockstep with `Dockerfile.jetson`) and
builds `isaac_ros_nvblox_utils` — expect a long first build.

### Bring up the GPU stack

```bash
docker compose -f docker/docker-compose.dev.yml --profile ros-gpu up --build
```

### ZED input without a physical camera

The Tier-2 image does not require a physical ZED 2i.  Drive ZED data via:

- **SVO playback** — pass an SVO recording to `zed_wrapper`:
  ```bash
  docker exec -it <container> \
      ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i \
          svo_file:=/data/my_recording.svo
  ```
- **Synthetic publisher** — run `tools/sim/zed_sim_publisher.py` inside the
  container for the same synthetic topics used by Tier 1:
  ```bash
  docker exec -it <container> \
      python3 /opt/nomad/tools/sim/zed_sim_publisher.py --rate 10
  ```

## See also

- [Deployment](deployment.md) — running the real Jetson Isaac ROS image on
  hardware (`Dockerfile.jetson`).
- [Getting Started](getting_started.md) — hardware-free dev with Edge Core alone
  (`Dockerfile.dev`) and the ArduPilot SITL stack.
