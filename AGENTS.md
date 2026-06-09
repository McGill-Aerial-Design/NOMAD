# NOMAD — Contributor & AI Agent Guide

NOMAD is a reusable drone **edge + ground-control** framework: a Python FastAPI
service that runs on an onboard companion computer (e.g. NVIDIA Jetson Orin Nano)
plus a C# Mission Planner plugin for the ground station. Capabilities (SLAM,
perception, payload, mission tasks) are intended to be added as discoverable
modules.

This document gives AI coding assistants and new contributors the project-specific
context that is otherwise expensive to rediscover. Treat it as a map, and always
verify against the source — the code is the source of truth.

---

## 1. Connection details (no hardcoded hosts)

There are **no hardcoded IPs in the repo.** Real addresses, keys, and host paths
live only in your local, gitignored `config/nomad.env` (copy it from
`config/nomad.env.example`) and in the Mission Planner plugin's settings.

| Property | Where it comes from |
|----------|---------------------|
| Companion computer address | `tailscale status`, or whatever you set in the plugin |
| SSH user | your device's login user (`JetsonSshUser` in the plugin; `JETSON_SSH_USER` env for scripts) |
| Edge Core port | `NOMAD_API_PORT` (default `8000`) |
| API docs URL | `http://<host>:<port>/docs` |
| Ground station address | `GCS_IP` in `config/nomad.env` (blank ⇒ discovered from Tailscale peers) |

```bash
# Discover peers on the tailnet
tailscale status

# SSH to the companion computer (substitute your user/host)
ssh "$JETSON_SSH_USER@<host>"

# Smoke-test the Edge Core API
curl -s "http://<host>:8000/health"
```

---

## 2. Project structure

```
NOMAD/
|-- edge_core/              # Python FastAPI server (runs on the companion computer)
|   |-- api.py              # App factory + API-key auth middleware
|   |-- api_routes/         # Route modules (system, services, terminal, streaming,
|   |                       #   vio, video_slam, isaac, calibration)
|   |-- main.py             # Entry point — boots the module registry
|   |-- core/               # Module SDK (NomadModule, ModuleRegistry, AppContext)
|   |-- services/           # state, mavlink/ (package), health_monitor,
|   |                       #   time_manager, video_stream_manager, payload_module,
|   |                       #   operational_mode, ipc, geospatial, ros/
|   |-- modules/            # Built-in capability modules (slam/isaac, payload/servo)
|   |-- ros_http_bridge/    # ROS→Edge Core bridge package (runs in Isaac container):
|   |                       #   node, coordinate_math, mesh_packer, mavlink_velocity, main
|
|-- docker/
| |-- Dockerfile.dev # x86_64 dev/sim image (no CUDA/ZED)
| |-- docker-compose.dev.yml # Hardware-free dev stack (Edge Core + ArduPilot SITL)
|
|-- mission_planner/src/ # C# plugin (runs on the Windows ground station)
|   |-- NOMADPlugin.cs               # Plugin entry point
|   |-- NOMADMainScreen.cs           # Main plugin host screen
|   |-- NOMADViewBase.cs             # Base view class (view registration)
|   |-- NOMADConfig.cs               # Plugin configuration model
|   |-- ... task/video/health/terminal/SLAM3D views + panels
|
|-- scripts/
|   |-- nomad                        # Single CLI dispatcher (start|stop|restart|status|logs)
|   |-- lib/common.sh                # Shared service-script helpers
|   |-- services/                    # One script per service
|   |-- build/                       # Build / compile helpers
|   |-- setup/                       # Provisioning + setup scripts
|   |-- dev/                         # Development tools + ad-hoc diagnostics
|   |-- hardware/                    # Hardware test utilities (incl. joystick.py)
|
|-- infra/systemd/                   # One systemd unit per service + install.sh
|   |                                # (units are templated; install.sh fills in
|   |                                #  the service user + paths from config/nomad.env)
|-- transport/mavlink_router/        # mavlink-router config
|-- tailscale/                       # VPN configuration and managers
|-- config/
|   |-- nomad.env.example            # Template config (committed)
|   |-- nomad.env                    # Your real runtime config (GITIGNORED)
```

---

## 3. Key configuration file

**`config/nomad.env`** is the single source of truth for runtime config. It is
**gitignored**; copy the committed template and edit it:

```bash
cp config/nomad.env.example config/nomad.env
# edit values, then:
nomad restart all       # or: sudo systemctl restart nomad.target
```

Every service script and the systemd units read it via `EnvironmentFile=`. It
holds service autostart flags (`NOMAD_AUTOSTART_*`), host paths, ports, auth
tokens, and per-service tuning. Generate an API key with
`python -c "import secrets; print(secrets.token_hex(32))"` and set the same value
in the plugin.

---

## 4. Edge Core API endpoints

> The API is defined in `edge_core/api.py` and `edge_core/api_routes/`. Only key
> endpoints are listed; always verify the source for the latest parameters.

### System / Network
- `GET /` · `GET /health` · `GET /health/detailed` · `GET /status`
- `GET /api/services/status` — process/service status
- `GET /network/status` · `GET /network/ping/{host}`

> Task-specific routes (`/api/task/*`, `/api/detections`, spray, …) are **not**
> in the baseline — add them per deployment as `NomadModule`s.

### Navigation
- Autonomous velocity control is **not** an HTTP route: the ROS→Edge Core bridge
  streams nav2/nvblox `cmd_vel` straight to ArduPilot GUIDED mode over its own
  MAVLink link (`ros_http_bridge/mavlink_velocity.py`).

### Servo / Calibration (payload)
- `POST /api/servo/camera/tilt?angle={0-180}` · `POST /api/servo/channel/{channel}/pwm`
- `POST /api/calibration/imu/reset_biases` · `POST /api/calibration/zed/sensor-viewer/start`

### Isaac / SLAM / Video
- `GET /api/isaac/status` · `POST /api/isaac/start` · `POST /api/isaac/stop` · `/api/isaac/bridge/{start,stop}`
- `GET /api/slam/mesh` · `POST /api/slam/mesh/update` (internal token)
- `GET /api/video/bridges` · `POST /api/video/source` · `POST /api/video/overlay/{action}` · `GET /api/stream/info`

### Terminal / WebSockets
- `GET /api/terminal/commands` · `GET /api/terminal/logs` (whitelisted; admin key)
- `WS /ws/state` (10 Hz state)

---

## 5. Development workflows

### Run Edge Core locally without hardware (sim)
```bash
python -m edge_core.main --sim --no-vision --port 8000
# then open http://localhost:8000/docs
```

### Deploy to the companion computer
```bash
ssh "$JETSON_SSH_USER@<host>"
cd ~/NOMAD && git pull origin main
nomad restart all
```

### Build the Mission Planner plugin (Windows)
```powershell
cd NOMAD
.\scripts\build\build_plugin_windows.ps1
# Output: %LOCALAPPDATA%\Mission Planner\plugins\NOMADPlugin.dll
```

### Smoke-test the API
```bash
curl -s "http://<host>:8000/health"
curl -s "http://<host>:8000/network/status" | python -m json.tool
```

---

## 6. Ports reference

| Service | Port | Protocol | Location |
|---------|------|----------|----------|
| Edge Core API | 8000 | TCP | companion computer |
| MAVLink LTE/Tailscale | 14560 | UDP | companion → GCS |
| MAVLink RadioMaster | 14550 | UDP | GCS local radio link |
| MAVLink Plugin Router | 14600 | UDP | Mission Planner merged link |
| RTSP Video | 8554 | TCP | companion (MediaMTX) |
| SSH | 22 | TCP | companion computer |

---

## 7. Engineering conventions

**Prefer simple, human-understandable code.** Write for the reader, not the
compiler. Favour flat logic over deep nesting, descriptive names over terseness,
and straightforward data flow over clever abstractions. If a piece of code is
hard to explain in one sentence, it is too complex.

**No monolithic files.** No file shall exceed 800 lines. CI rejects pull requests
that add or modify source files breaching this limit. If a module is growing past
800 lines, split it into smaller focused files.

**Line length cap is 120 characters.** Run `pixi run fmt` to auto-format. CI
enforces this via ruff lint + format checks.

**Auto-format everything.** `pixi run fmt` runs ruff format on the entire Python
codebase. The pre-commit hook runs it automatically on staged files. There is no
excuse for formatting inconsistencies.

**No hardcoded secrets or hosts.** Never commit IPs, keys, emails, or absolute
user paths. Read them from `config/nomad.env` / environment variables, or use a
placeholder (`<host>`, `<jetson-ip>`) in docs. The repo must stay clean for
`git grep` of any real address.

**Pick one approach; don't build fallback chains.** Avoid
"Strategy 1 → Strategy 2 → Strategy 3" cascades. Analyze the problem, choose the
most reliable approach, and implement it properly with real error handling. If an
approach has a prerequisite (a service must be running), ensure the prerequisite
is met rather than coding around its absence.

**Match the surrounding code.** Follow existing naming, structure, and comment
density in the file you are editing.

---

## 7a. Commit message guidelines

Maximum width is 72 characters. Make one commit per logical change. Resist mixing
coding-style fixes into feature commits — keep style fixes in their own commits.

```
[part,sub-part] Short description in imperative

Longer explanation if needed. Skip a line between the subject
and body. Explain what and why, not how.
```

- Prefix (always lowercase) names the part and optional sub-part in brackets.
- Start the description with a capital letter and an imperative verb.
- Examples:

```
[sr1000_api,calib] Skip code 6 and 7 in frequency calibration
[phy] Inverse pulses order
[link] Fix coding style issues
[ranging] Add offsets to long range values
[application] Change radio count value to 2
```

Examples for this repo:

```
[edge_core] Add spray aim retry when depth sample is missing
[mission_planner] Fix WASD key release on panel exit
[docs] Genericize the lead-in and update module guide
```

---

## 8. Contribution workflow

NOMAD uses the **forking workflow** — see [CONTRIBUTING.md](CONTRIBUTING.md)
for the full guide: fork, branch naming, commit format, and merge request
process. The main repo stays branch-free; all development happens in forks.

---

## 9. Documentation index

The documentation site is built with MkDocs Material (`pixi run docs`).

| Topic | File |
|-------|------|
| Project overview | `README.md` |
| Getting Started (pixi + Docker sim) | `docs/getting_started.md` |
| Architecture | `docs/architecture.md` |
| Writing a Module (plugin SDK) | `docs/writing_a_module.md` |
| Deployment (Jetson image + systemd) | `docs/deployment.md` |
| Configuration reference | `docs/configuration.md` |
| API Reference | `docs/api_reference.md` |
| Tailscale setup | `infra/tailscale/SETUP.md` |
| Edge Core | `edge_core/README.md` |
| Mission Planner plugin | `mission_planner/README.md` |
