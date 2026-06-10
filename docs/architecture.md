# Architecture

NOMAD is divided into three logical domains:

- **Transport (A)** — connectivity and MAVLink routing
- **Edge Core (B)** — the onboard Python FastAPI service
- **Mission Planner Plugin (C)** — the ground station C# plugin

## System overview

```
            DRONE (companion computer)                         GROUND STATION
 ┌───────────────────────────────────────────────┐        ┌────────────────────┐
 │                                                │        │  Mission Planner   │
 │  Flight Controller (ArduPilot)                 │        │   + NOMAD plugin    │
 │        │ UART (MAVLink)                        │        │        (C#)         │
 │        ▼                                       │        └─────────┬──────────┘
 │  mavlink-router ──────── UDP/VPN ──────────────┼── Tailscale ─────┤  REST/WS
 │        │ localhost UDP                          │   (4G/WiFi)      │  + MAVLink
 │        ▼                                       │                  │
 │  ┌──────────────── Edge Core (FastAPI) ──────┐ │◄─── HTTP/WS ─────┘
 │  │  module registry: configure→routes→       │ │
 │  │  start→stop  (ASGI lifespan)              │ │
 │  │  services: state · mavlink · health ·     │ │
 │  │  video · payload · time · geo             │ │
 │  │  api_routes: system · vio · slam · isaac  │ │
 │  └───────────────▲───────────────────────────┘ │
 │     HTTP (loopback, internal token)            │
 │  ┌──────────────┴── ROS-HTTP bridge ─────────┐ │
 │  │  ZED / nvblox / nav2  →  pose, mesh,       │ │
 │  │  cmd_vel → MAVLink GUIDED (direct link)    │ │
 │  └────────────────────────────────────────────┘ │
 └───────────────────────────────────────────────┘
```

Module lifecycle: each module is discovered via the `nomad.modules`
entry-point group, then driven through `configure(ctx)` → `register_routes(app)`
→ `start()` → `stop()` (start/stop run on the ASGI lifespan; see
[Writing a Module](writing_a_module.md)).

## Domains

### A — Transport Layer

Responsible for all communication between the flight controller (FC), the
companion computer, and the ground station:

- `mavlink-router` on the companion computer routes MAVLink traffic between FC
  (UART), Edge Core (localhost UDP), and the ground station (UDP over VPN/radio).
- Tailscale VPN provides secure connectivity over 4G/LTE or WiFi.
- ELRS radio links provide a low-latency control backup independent of the
  companion computer.

### B — Edge Core

A Python FastAPI service running on the companion computer (e.g. NVIDIA Jetson
Orin Nano). It owns:

- **REST/WebSocket API** — system status, network, servo/calibration, SLAM mesh,
  video, and Isaac container control (task-specific routes are added per
  deployment as modules)
- **State management** — thread-safe singleton (`services/state.py`)
- **MAVLink interface** — telemetry and command routing to/from the FC
  (`services/mavlink/` package: `connection`, `commands`, `module`)
- **Direct nav velocity** — nav2/nvblox `cmd_vel` is streamed straight to
  ArduPilot GUIDED mode by the ROS→Edge Core bridge over its own MAVLink link
  (`ros_http_bridge/mavlink_velocity.py`), with clamping, VIO-freshness gating,
  and a command-timeout watchdog — no HTTP hop through Edge Core
- **Video pipeline** — source switching, overlays, RTSP streaming
  (`services/video_stream_manager.py`)
- **Payload control** — servo/relay commands (`services/payload_module.py`)
- **Health monitoring** — CPU/GPU temperature, memory, disk, network
  (`services/health_monitor.py`)
- **Module registry** — discovers and loads capabilities via the `nomad.modules`
  entry-point group (see [Writing a Module](writing_a_module.md))

### C — Mission Planner Plugin

A C# .NET Framework 4.8 plugin that runs inside Mission Planner on the Windows
ground station. It provides:

- Dashboard with system overview and quick actions
- Flight-boundary / geofence configuration and upload
- Embedded RTSP video player and 3D SLAM viewer
- Remote terminal access to the companion computer
- Dual-link MAVLink failover management
- Real-time health and network monitoring
- Config profiles with an active-profile indicator (synced from `scripts/profile.py`)

## Data flow

```
FC UART → mavlink-router → Edge Core (localhost UDP)
                         → Ground Station (UDP over Tailscale/ELRS)

ZED camera → Isaac ROS (Docker) → API state → GCS
                                → Video bridge → MediaMTX RTSP → GCS

Mission Planner plugin → HTTP API → Edge Core → MAVLink → FC (commands)
                       ← WebSocket ← Edge Core ← FC (telemetry)
```

## Module system

Capabilities are added as discoverable modules that register via the
`nomad.modules` entry-point group. Each module implements a lifecycle:

1. `configure(ctx)` — receive an `AppContext` with core service references
2. `register_routes(app)` — add FastAPI routes
3. `start()` — begin background tasks
4. `stop()` — clean up resources

Built-in modules include SLAM (Isaac ROS/nvblox bridge), perception
(detectors/target localizer), and payload (servo/spray).

See [Writing a Module](writing_a_module.md) for the full guide.

## Performance considerations

### Video latency

Current pipeline: `ZED → ROS → Python bridge → FFmpeg → MediaMTX → RTSP → GCS`

Each hop adds latency. If sub-200ms glass-to-glass latency is required, a direct
GStreamer pipeline from the ZED wrapper to RTSP can bypass the Python bridge.

### Resource contention on Jetson

The Jetson Orin Nano runs multiple concurrent workloads: Docker (Isaac ROS),
Python Edge Core, software H.264 encoding (no NVENC on Orin Nano), and
mavlink-router. The health monitor (`health_monitor.py`) tracks thermals and can
trigger alerts. Under thermal pressure, consider reducing Isaac ROS update rates.
