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
| `ISAAC_WORKSPACE` | `~/workspaces/isaac_ros-dev` | Isaac ROS workspace |
| `ISAAC_ROS_DOMAIN_ID` | `0` | ROS2 domain ID |
| `ZED_CAMERA_MODEL` | `zed2i` | ZED camera model |
| `ZED_GRAB_RESOLUTION` | `HD720` | ZED grab resolution |
| `ZED_DEPTH_CONFIDENCE` | `95` | ZED depth confidence threshold |
| `ZED_DEPTH_MODE` | `NEURAL_LIGHT` | ZED depth backend |

### Video bridge

| Variable | Default | Description |
|----------|---------|-------------|
| `VIDEO_BRIDGE_STREAM_PATH` | `primary` | RTSP stream path |
| `VIDEO_BRIDGE_WIDTH` | `640` | Stream width |
| `VIDEO_BRIDGE_HEIGHT` | `360` | Stream height |
| `VIDEO_BRIDGE_FPS` | `15` | Stream framerate |
| `VIDEO_BRIDGE_BITRATE` | `800` | Stream bitrate (kbps) |

### Detection

| Variable | Default | Description |
|----------|---------|-------------|
| `NOMAD_DETECTIONS_AUTO_START` | `false` | Auto-start detection pipeline |
| `NOMAD_DETECTOR_INTERVAL_S` | `0.1` | Detector loop interval (seconds) |
| `NOMAD_DETECTOR_MAX_WIDTH` | `960` | Downsample width for detection |

### nvblox

| Variable | Default | Description |
|----------|---------|-------------|
| `NOMAD_PROBE_NVBLOX` | `false` | Probe for nvblox at startup |
| `NOMAD_ENABLE_NVBLOX_MESH` | `false` | Enable mesh streaming |

### Module control

| Variable | Default | Description |
|----------|---------|-------------|
| `NOMAD_MODULES` | *(blank)* | Module allow-list (comma-separated; blank = all) |
| `NOMAD_ENABLE_<MODULE>` | `true` | Disable a specific module by name |
