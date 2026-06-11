# Configuration

NOMAD uses `config/nomad.env` as the single source of truth for runtime
configuration. This file is **gitignored** — copy the committed template and
edit it for your deployment:

```bash
cp config/nomad.env.example config/nomad.env
# edit values, then restart services
```

## Profiles

Rather than hand-editing `config/nomad.env`, use a named **profile** — a
complete env file in `config/profiles/`:

| Profile | Use |
|---------|-----|
| `dev` | Minimal Edge Core, sim mode, no vision/containers (API dev, CI) |
| `drone` | Real Jetson + ZED2i + CubePilot (production; secrets set locally) |

```bash
pixi run profile-load dev    # copy profile -> config/nomad.env
pixi run profile-show        # active profile
pixi run profile-list        # all profiles
pixi run profile-save mine   # snapshot current config as a new profile
```

Loading a profile also **syncs the Mission Planner plugin** config
(`%LOCALAPPDATA%\Mission Planner\plugins\nomad_config.json`): the `JetsonApiKey`,
endpoint, and an `ActiveProfile` marker (shown in the plugin's sidebar). Override
the plugin path with `NOMAD_MP_CONFIG`. The same actions are available in VS Code
under **Run Task → "Profile: …"**.

> The committed `dev`/`sim` profiles use the dev key `nomad-dev-key`. Real
> deployments keep their `NOMAD_API_KEY` / `NOMAD_INTERNAL_TOKEN` local
> (untracked) — never commit production secrets.

## File reference

### Service autostart

Controls which services are started by `nomad start all` and at boot.

| Variable | Default | Description |
|----------|---------|-------------|
| `NOMAD_AUTOSTART_EDGE_CORE` | `true` | Edge Core FastAPI service |
| `NOMAD_AUTOSTART_MAVLINK_ROUTER` | `true` | MAVLink routing |
| `NOMAD_AUTOSTART_MEDIAMTX` | `true` | RTSP server |
| `NOMAD_AUTOSTART_ISAAC_ROS_CONTAINER` | `true` | Isaac ROS container |
| `NOMAD_AUTOSTART_ZED_WRAPPER` | `true` | ZED camera ROS wrapper |
| `NOMAD_AUTOSTART_ROS_HTTP_BRIDGE` | `true` | ROS ↔ HTTP bridge |
| `NOMAD_AUTOSTART_VIDEO_BRIDGE` | `true` | Video encoding bridge |
| `NOMAD_AUTOSTART_NVBLOX` | `false` | nvblox (manual start only) |
| `NOMAD_AUTOSTART_HEALTH_MONITOR` | `true` | Jetson health monitor module |
| `NOMAD_AUTOSTART_TIME_SYNC` | `true` | Time synchronization module |

### Paths

| Variable | Default | Description |
|----------|---------|-------------|
| `NOMAD_REPO_ROOT` | `/home/<user>/NOMAD` | Repository checkout root |
| `NOMAD_LOG_DIR` | `/home/<user>/nomad_logs` | Service log output |
| `NOMAD_RUN_DIR` | `/run/nomad` | Runtime PID/socket files |
| `NOMAD_DATA_DIR` | `$NOMAD_REPO_ROOT/data` | Data storage root |
| `NOMAD_MISSION_LOG_DIR` | `$NOMAD_DATA_DIR/mission_logs` | Mission log output |
| `NOMAD_VENV` | `$NOMAD_REPO_ROOT/venv` | Python virtualenv |

### Network

| Variable | Default | Description |
|----------|---------|-------------|
| `NOMAD_API_HOST` | `0.0.0.0` | API bind address |
| `NOMAD_API_PORT` | `8000` | API port |
| `NOMAD_API_URL` | `http://localhost:8000` | Local service-script API URL |
| `NOMAD_DOCKER_HOST_IP` | `172.17.0.1` | Host address reachable from Docker containers |
| `GCS_IP` | *(blank)* | Ground station Tailscale IP (auto-discovered if blank) |
| `GCS_EXTRA_IPS` | *(blank)* | Additional GCS IPs for MAVLink LTE |
| `GCS_PORT_LTE` | `14560` | LTE MAVLink port on GCS |
| `GCS_PORT_LOCAL` | `14550` | Local MAVLink port on GCS |

### Authentication

| Variable | Default | Description |
|----------|---------|-------------|
| `NOMAD_API_KEY` | *(blank)* | API key (generate with `python -c "import secrets; print(secrets.token_hex(32))"`) |
| `NOMAD_INTERNAL_TOKEN` | *(blank)* | Internal service token |
| `NOMAD_ALLOW_INSECURE_REMOTE` | `false` | Allow remote access without API key |
| `NOMAD_ENABLE_TERMINAL_EXEC` | `false` | Enable arbitrary terminal execution |
| `NOVNC_VNC_PASSWORD` | *(blank)* | noVNC VNC password |

### MAVLink

| Variable | Default | Description |
|----------|---------|-------------|
| `MAVLINK_UART_DEV` | `/dev/ttyACM0` | FC serial device |
| `MAVLINK_UART_BAUD` | `921600` | FC serial baud rate |

### Servo / payload

| Variable | Default | Description |
|----------|---------|-------------|
| `NOMAD_ENABLE_SERVOS` | `true` | Enable servo/relay control |
| `NOMAD_STEREO_FX_BASELINE_PX_M` | `32.0` | Stereo parallax correction factor (ZED 2i) |

### MediaMTX

| Variable | Default | Description |
|----------|---------|-------------|
| `RTSP_PORT` | `8554` | RTSP server port |
| `MEDIAMTX_CONFIG` | `$NOMAD_REPO_ROOT/infra/mediamtx.yml` | MediaMTX config path |
| `MEDIAMTX_BIN` | `~/bin/mediamtx` | MediaMTX binary path |

### Isaac ROS / ZED

| Variable | Default | Description |
|----------|---------|-------------|
| `ISAAC_CONTAINER_NAME` | `nomad_isaac_ros` | Docker container name |
| `ISAAC_IMAGE_NAME` | `nomad-isaac-ros:latest` | Docker image tag |
| `ISAAC_IMAGE_FALLBACK` | `isaac_ros_dev-aarch64` | Existing image name accepted by provisioning |
| `ISAAC_WORKSPACE` | `~/workspaces/isaac_ros-dev` | Isaac ROS workspace |
| `ISAAC_ROS_DOMAIN_ID` | `0` | ROS2 domain ID |
| `ZED_CAMERA_MODEL` | `zed2i` | ZED camera model |
| `ZED_CAMERA_NAME` | `zed` | ZED ROS node camera name |
| `ZED_PUB_RESOLUTION` | `NATIVE` | Published image resolution mode |
| `ZED_PUB_DOWNSCALE_FACTOR` | `2.0` | Downscale factor when using custom publish resolution |
| `ZED_PUBLISH_RAW` | `true` | Publish raw camera image topics |
| `ZED_PUBLISH_LEFT_RIGHT` | `true` | Publish left/right image topics |
| `ZED_PUBLISH_MAG` | `true` | Publish magnetometer data |
| `ZED_GRAB_RESOLUTION` | `HD720` | ZED grab resolution |
| `ZED_DEPTH_CONFIDENCE` | `95` | ZED depth confidence threshold |
| `ZED_DEPTH_TEXTURE_CONF` | `90` | ZED texture-confidence threshold |
| `ZED_DEPTH_MODE` | `NEURAL_LIGHT` | ZED depth backend |
| `ZED_READY_TIMEOUT_S` | `300` | Startup wait for ZED odometry |

### ROS-HTTP bridge

| Variable | Default | Description |
|----------|---------|-------------|
| `ROS_HTTP_BRIDGE_RATE` | `5` | Bridge publish/poll rate |
| `ROS_HTTP_BRIDGE_VIO_TOPIC` | `/zed/zed_node/odom` | VIO odometry topic |
| `ROS_HTTP_BRIDGE_MAG_TOPIC` | `/zed/zed_node/imu/mag` | Magnetometer topic |
| `ROS_HTTP_BRIDGE_TRANSPORT` | `http` | Bridge transport mode |
| `NOMAD_ROS_ROOT` | `/opt/ros/humble` | ROS install root inside the container |
| `NOMAD_ISAAC_WORKSPACE` | `/workspaces/isaac_ros-dev` | Isaac ROS workspace inside the container |
| `NOMAD_VIO_MAX_AGE_S` | `1.0` | Maximum VIO age before command rejection |

### Video bridge

| Variable | Default | Description |
|----------|---------|-------------|
| `VIDEO_BRIDGE_STREAM_PATH` | `primary` | RTSP stream path |
| `VIDEO_BRIDGE_WIDTH` | `640` | Stream width |
| `VIDEO_BRIDGE_HEIGHT` | `360` | Stream height |
| `VIDEO_BRIDGE_FPS` | `15` | Stream framerate |
| `VIDEO_BRIDGE_BITRATE` | `800` | Stream bitrate (kbps) |
| `VIDEO_RELAY_HTTP_PORT` | `9200` | Bridge control HTTP port |
| `NOMAD_ENABLE_VIDEO` | `true` | Enable the video stream module |
| `NOMAD_DEFAULT_VIDEO_TOPIC` | `/zed/zed_node/rgb/color/rect/image` | Default source topic |
| `NOMAD_RTSP_URL` | *(blank)* | Explicit RTSP URL override |

### Detection

| Variable | Default | Description |
|----------|---------|-------------|
| `NOMAD_DETECTIONS_AUTO_START` | `false` | Auto-start detection pipeline |
| `NOMAD_DETECTOR_INTERVAL_S` | `0.1` | Detector loop interval (seconds) |
| `NOMAD_DETECTOR_MAX_WIDTH` | `960` | Downsample width for detection |

### nvblox

| Variable | Default | Description |
|----------|---------|-------------|
| `NOMAD_ENABLE_NVBLOX_MESH` | `false` | Enable mesh streaming |

### Module control

| Variable | Default | Description |
|----------|---------|-------------|
| `NOMAD_MODULES` | *(blank)* | Module allow-list (comma-separated; blank = all) |
| `NOMAD_ENABLE_<MODULE>` | `true` | Disable a specific module by name |

### Network monitoring

| Variable | Default | Description |
|----------|---------|-------------|
| `NOMAD_ENABLE_NETWORK_MONITOR` | `true` | Enable the Tailscale/LTE network module |
| `NOMAD_LTE_CONNECTION` | `NOMAD-LTE` | NetworkManager profile owning the LTE modem |
| `GCS_IP` | *(blank)* | GCS Tailscale IP used for reachability checks |

### Health and MAVLink safety

| Variable | Default | Description |
|----------|---------|-------------|
| `NOMAD_HEALTH_BROADCAST_INTERVAL_S` | `2.0` | MAVLink health broadcast interval |
| `NOMAD_MAVLINK_ENDPOINT` | `127.0.0.1:14550` | Edge Core MAVLink endpoint |
| `NOMAD_MAVLINK_DISCONNECT_TIMEOUT_S` | `3.0` | MAVLink disconnect timeout |
| `NOMAD_BRIDGE_MAVLINK_ENDPOINT` | `127.0.0.1:14552` | ROS bridge velocity MAVLink endpoint |
| `NOMAD_FENCE_POLYGON` | *(blank)* | Optional NOMAD-side keep-in polygon |
| `NOMAD_FENCE_MARGIN_M` | `2.0` | Keep-in polygon margin |

### Optional uploads and misc

| Variable | Default | Description |
|----------|---------|-------------|
| `GDRIVE_FOLDER_ID` | *(blank)* | Optional Google Drive upload folder |
| `PYTHONUNBUFFERED` | `1` | Force unbuffered Python service logs |
