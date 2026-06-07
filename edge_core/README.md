# Edge Core

Onboard companion-computer service (Python 3.13, FastAPI). Runs on the Jetson
on the drone, or hardware-free in sim mode on any machine.

## Layout

| Path | Purpose |
|------|---------|
| `main.py` | Entry point; boots the module registry and FastAPI server |
| `api.py` | App factory, API-key auth middleware, app.state wiring |
| `api_context.py` / `api_models.py` | Route DI context + Pydantic models |
| `core/` | Module SDK: `NomadModule`, `ModuleRegistry`, `AppContext` (entry-point discovery) |
| `services/state.py` | Thread-safe state manager singleton |
| `services/mavlink/` | MAVLink package — `connection` (RX/telemetry), `commands` (TX), `module` (SDK wrapper) |
| `services/health_monitor.py` | Jetson hardware monitoring (CPU/GPU/temp/mem/disk) |
| `services/time_manager.py` | NTP/GPS time synchronization |
| `services/video_stream_manager.py` | Video bridge / overlay / source switching |
| `services/payload_module.py` | Servo / relay payload module |
| `services/operational_mode.py` | Operational mode state machine |
| `services/ipc.py` | ZeroMQ IPC helpers for high-rate ROS data |
| `services/geospatial.py` · `logging_service.py` | Coordinate conversions · mission logging |
| `services/ros/` | ROS2-side helper nodes (e.g. video bridge) |
| `api_routes/` | Route modules (system, services, terminal, streaming, video_slam, isaac, isaac_sim, calibration) |
| `modules/` | Built-in capability modules (slam/isaac, payload/servo) |
| `ros_http_bridge/` | ROS→Edge Core bridge package (runs inside the Isaac container) — see its own notes |

The bridge package (`ros_http_bridge/`) splits the old monolith into `node.py`
(the ROS2 node), `coordinate_math.py` (pure quaternion/NED helpers),
`mesh_packer.py` (voxel serialization + background send), `mavlink_velocity.py`
(direct `cmd_vel`→ArduPilot GUIDED velocity stream), and `main.py` (CLI).

## Quick start

```bash
pixi run dev          # Edge Core in sim mode (no hardware) on http://localhost:8000
pixi run dev-up       # Docker: Edge Core + ArduPilot SITL (no Isaac Sim)
pixi run test-api     # exercise every REST endpoint
```

Open `http://localhost:8000/docs` for the interactive Swagger UI.

## Modules

Capabilities load via the `nomad.modules` entry-point group (see
`pyproject.toml`) in dependency order. Each implements the `NomadModule`
lifecycle: `configure(ctx)` → `register_routes(app)` → `start()` → `stop()`.
Enable/disable with `NOMAD_ENABLE_<MODULE>` / `NOMAD_MODULES`.

## API endpoint groups

The running server's `/docs` is authoritative; groups depend on which modules
are loaded. The general baseline exposes:

| Group | Prefix | Key endpoints |
|-------|--------|---------------|
| System | `/`, `/health`, `/health/detailed`, `/status` | Service info, health, system state |
| Services | `/api/services/status` | Process/service status |
| Network | `/network/*` | Tailscale status, reconnect, ping |
| Servo / Calibration | `/api/servo/*`, `/api/calibration/*` | Camera tilt, channel PWM, IMU/ZED calibration |
| SLAM | `/api/slam/mesh`, `/api/slam/mesh/update` | nvblox voxel mesh stream (general, task-agnostic) |
| Video | `/api/video/*`, `/api/stream/*` | Source switching, overlays, bridges, stream info |
| Isaac | `/api/isaac/*`, `/api/sim/*` | Isaac ROS / Isaac Sim container lifecycle |
| Terminal | `/api/terminal/*` | Whitelisted commands, logs (admin) |
| VIO (internal) | `/api/vio/update` | Bridge → Edge Core pose ingest (internal token) |

> Task-specific routes (`/api/task/1`, `/api/task/2`, `/api/detections`, …) are
> intentionally **not** part of this baseline — add them per deployment as
> `NomadModule`s.
