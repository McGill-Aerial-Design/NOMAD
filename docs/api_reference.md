# API Reference

Edge Core exposes a REST API with interactive documentation at `/docs` (Swagger
UI) when the server is running. This page summarizes the endpoint groups in the
general baseline.

> The authoritative source is the running server's `/docs`. Endpoints vary with
> the modules loaded. Task-specific routes (e.g. `/api/task/*`, `/api/detections`)
> are not part of the baseline — add them per deployment as `NomadModule`s.
>
> Tip: `pixi run test-api` exercises every live endpoint.

## Authentication

If `NOMAD_API_KEY` is set, all routes except `/`, `/health`, `/docs`, `/redoc`,
and `/openapi.json` require an `X-API-Key` header. With no key configured, only
loopback clients are allowed unless `NOMAD_ALLOW_INSECURE_REMOTE=true`. The dev
profile ships a committed `nomad-dev-key` so the stack works out of the box.

## System

| Method | Path | Description |
|--------|------|-------------|
| GET | `/` | Root info |
| GET | `/health` | Basic health check (exempt from auth) |
| GET | `/health/detailed` | Detailed health (CPU, GPU, memory, disk) |
| GET | `/status` | Full system status (telemetry snapshot) |
| GET | `/api/services/status` | Process/service status |

## Network

| Method | Path | Description |
|--------|------|-------------|
| GET | `/network/status` | Tailscale and network status |
| GET | `/network/ping/{host}` | Ping a host |

## Navigation

Autonomous velocity control is **not** exposed over HTTP. The ROS→Edge Core
bridge streams nav2/nvblox `cmd_vel` straight to ArduPilot GUIDED mode over its
own MAVLink link (`ros_http_bridge/mavlink_velocity.py`), with all flight-safety
gating (clamp, VIO freshness, armed+GUIDED, watchdog) owned there.

## Servo & Calibration

| Method | Path | Description |
|--------|------|-------------|
| POST | `/api/servo/camera/tilt` | Set camera tilt angle (`?angle=0-180`) |
| POST | `/api/servo/channel/{channel}/pwm` | Set a raw servo channel PWM |
| POST | `/api/servo/shooter/arm` | Arm the water-shooter release interlock (short window, consumed per attempt) |
| POST | `/api/servo/shooter/trigger` | Fire the water shooter (`?duration_ms=&relay_number=`); requires a prior arm |
| POST | `/api/calibration/imu/reset_biases` | Reset IMU biases |
| POST | `/api/calibration/zed/sensor-viewer/start` | Launch ZED sensor viewer |

## SLAM

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/slam/mesh` | Cached nvblox voxel mesh (`?format=summary\|json`) |
| POST | `/api/slam/mesh/update` | Mesh ingest from the bridge (internal token) |

## Video

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/video/bridges` | List video bridges |
| POST | `/api/video/bridges/start` | Start a video bridge |
| POST | `/api/video/source` | Switch video source |
| POST | `/api/video/overlay/{action}` | Toggle overlay rendering |
| GET | `/api/stream/info` | Stream/RTSP info |

## Isaac ROS

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/isaac/status` | Isaac ROS container status |
| POST | `/api/isaac/start` · `/api/isaac/stop` | Container lifecycle |
| POST | `/api/isaac/bridge/start` · `/api/isaac/bridge/stop` | ROS-HTTP bridge lifecycle |

## Terminal (admin)

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/terminal/commands` | List whitelisted commands |
| GET | `/api/terminal/logs` | Service logs |
| POST | `/api/terminal/exec` · `/api/terminal/run` | Run a whitelisted command (requires API key) |

## Internal ingest

| Method | Path | Description |
|--------|------|-------------|
| POST | `/api/vio/update` | Bridge → Edge Core VIO pose (internal token, loopback) |

## WebSockets

| Path | Rate | Description |
|------|------|-------------|
| `WS /ws/state` | 10 Hz | Real-time system state |
