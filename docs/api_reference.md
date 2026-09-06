# Transitional API Reference

This page documents the current Python Edge Core REST service only. It is not the
NOMAD product boundary and will be deleted with the Python service after the C++
client boundary passes its migration gates.

Edge Core exposes a REST API with interactive documentation at `/docs` (Swagger
UI) when the server is running. This page summarizes the endpoint groups in the
current baseline.

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

Autonomous velocity control is **not** exposed over HTTP. The C++ ROS 2 adapter
(`ros2/nomad_ros`) translates `cmd_vel` into the C++ core's
`Vehicle::set_velocity` with all flight-safety gating (clamp, VIO freshness,
armed+GUIDED, watchdog) owned there.

## Vehicle commands

Vehicle command paths were removed from the REST surface in the C++ cutover
(2026-09-05). Servo/relay/payload/motion commands belong to the C++ core CLI
(`nomad servo|relay|motor-test|gimbal-config|payload-demo|goto ...`) and the
Mission Planner `NomadCoreClient`; payloads are modular client profiles over
generic ArduPilot outputs, not core concepts.

## Calibration

| Method | Path | Description |
|--------|------|-------------|
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
| GET | `/api/video/topics` | List ROS image topics available to the video bridge |
| POST | `/api/video/bridges/start` | Start a video bridge |
| POST | `/api/video/source` | Switch video source |
| GET | `/api/video/overlay/status` | Current overlay rendering state |
| POST | `/api/video/overlay/{action}` | Toggle overlay rendering |
| GET | `/api/stream/info` | Stream/RTSP info |

## Isaac ROS

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/isaac/status` | Isaac ROS container status |
| POST | `/api/isaac/start` · `/api/isaac/stop` | Container lifecycle |
| POST | `/api/isaac/bridge/start` · `/api/isaac/bridge/stop` | C++ ROS adapter node lifecycle (in-container `nomad_vehicle_node` via `nomad-ros-vehicle.service`) |

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
