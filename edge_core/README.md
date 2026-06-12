# Edge Core

Onboard companion-computer service (Python 3.13, FastAPI). Runs on the Jetson
on the drone, or hardware-free in sim mode on any machine.

## Layout

| Path | Purpose |
|------|---------|
| `main.py` / `cli.py` / `runtime.py` | Entry point · argparse · run/cleanup/signal handling |
| `api.py` | App factory only (`create_app`) |
| `api_auth.py` / `api_cors.py` | API-key auth middleware + settings · CORS setup |
| `api_state.py` / `api_route_registry.py` | `app.state` wiring · route registration |
| `api_context.py` / `api_models.py` | Route DI context + Pydantic models |
| `env.py` | Shared environment parsing helpers (`env_bool`, `env_secret`) |
| `core/` | Module SDK: `NomadModule`, `ModuleRegistry`, `AppContext` (entry-point discovery) |
| `safety/` | Safety-critical decision core (pure, 100% branch coverage enforced) |
| `platform/` | Platform-specific workarounds (Jetson library preload) |
| `services/state.py` | Thread-safe state manager singleton |
| `services/mavlink/` | MAVLink package — `connection` (RX/telemetry), `commands` (TX), `module` (SDK wrapper) |
| `services/health_monitor.py` | Jetson hardware monitoring (CPU/GPU/temp/mem/disk) |
| `services/time_manager.py` | NTP/GPS time synchronization |
| `services/video_stream_manager.py` / `video_module.py` | Video bridge / overlay / source switching |
| `services/payload_module.py` | Servo / relay payload module |
| `services/network_module.py` | Tailscale / network status |
| `services/geospatial.py` | Coordinate conversions |
| `services/nav/` · `services/ros/` | Navigation helpers · ROS2-side helper nodes |
| `api_routes/` | Route modules (system, services, terminal, streaming, vio, video_slam, isaac, calibration) |
| `modules/` | Built-in capability modules (slam/isaac, payload/servo) |
| `ros_http_bridge/` | ROS→Edge Core bridge package (runs inside the Isaac container) — see its own notes |

The bridge package (`ros_http_bridge/`) splits the old monolith into `node.py`
(the ROS2 node), `coordinate_math.py` (pure quaternion/NED helpers),
`mesh_packer.py` (voxel serialization + background send), `mavlink_velocity.py`
(direct `cmd_vel`→ArduPilot GUIDED velocity stream), `http_client.py`
(keep-alive HTTP transport to Edge Core), and `main.py` (CLI).

## Quick start

```bash
pixi run dev          # Edge Core in sim mode (no hardware) on http://localhost:8000
pixi run dev-up       # Docker: Edge Core + ArduPilot SITL (hardware-free)
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
| Isaac | `/api/isaac/*` | Isaac ROS container + ROS-HTTP bridge lifecycle |
| Terminal | `/api/terminal/*` | Whitelisted commands, logs (admin) |
| VIO (internal) | `/api/vio/update` | Bridge → Edge Core pose ingest (internal token) |

> Task-specific routes (`/api/task/1`, `/api/task/2`, `/api/detections`, …) are
> intentionally **not** part of this baseline — add them per deployment as
> `NomadModule`s.
