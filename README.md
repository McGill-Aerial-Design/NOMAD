# NOMAD

**A reusable drone edge + ground-control framework.**

NOMAD pairs a Python FastAPI service running on an onboard companion computer
(e.g. NVIDIA Jetson Orin Nano) with a C# Mission Planner plugin for the ground
station. It handles MAVLink telemetry/commands, navigation, video streaming,
health monitoring, payload actuation, and ROS2 bridges — and is designed so new
capabilities (SLAM, perception, payloads, mission tasks) are added as
discoverable modules.

# Quick start (sim, no hardware)

## Install pixi (https://pixi.sh) — one of:
    Linux/macOS:  curl -fsSL https://pixi.sh/install.sh | bash
    Windows:      iwr -useb https://pixi.sh/install.ps1 | iex
```bash
git clone <repo-url> && cd NOMAD
pixi run dev              # Edge Core sim on http://localhost:8000

# Or the full hardware-free Docker stack (Edge Core + ArduPilot SITL):
pixi run dev-up           # build + start (detached)
pixi run test-api         # smoke-test every REST endpoint
pixi run dev-down         # stop

# Switch environments (sim / drone / dev) — also syncs the GCS plugin:
pixi run profile-load dev
```

## Repository structure

```
NOMAD/
├── edge_core/             # Python FastAPI service (companion computer)
│   ├── core/              # Module SDK: registry, AppContext, lifecycle
│   ├── services/          # state, mavlink/, health, video, payload, IPC, geo
│   ├── api_routes/        # Route modules (system, vio, video_slam, isaac, calibration, …)
│   ├── modules/           # Built-in pluggable modules (slam, payload)
│   └── ros_http_bridge/   # ROS→Edge Core bridge package (runs in Isaac container)
├── mission_planner/src/   # C# Mission Planner plugin
├── docker/                # Dockerfile.dev + hardware-free dev compose (Edge Core + SITL)
├── scripts/               # Service mgmt, profiles, build, dev tools
├── infra/                 # systemd units, transport (mavlink-router), tailscale
└── config/                # nomad.env (+ profiles/) — runtime config
```

## Documentation

Full documentation is available at the [docs site](docs/index.md) (served
locally via `pixi run docs`). Key pages:

- [Getting Started](docs/getting_started.md) — Pixi env, Docker sim, first run
- [Architecture](docs/architecture.md) — System design and data flow
- [Writing a Module](docs/writing_a_module.md) — Plugin SDK guide
- [Deployment](docs/deployment.md) — Jetson all-in-one image, systemd
- [Configuration](docs/configuration.md) — `config/nomad.env` reference
- [API Reference](docs/api_reference.md) — Edge Core endpoints

## Forking workflow

Contributors work in **personal forks** — no branches are pushed directly to the main repo.
See [CONTRIBUTING.md](CONTRIBUTING.md) for the full workflow.

## License

Apache 2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE).
