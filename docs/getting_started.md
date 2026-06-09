# Getting Started

NOMAD provides two development paths:

1. **Pixi environment** — hardware-free sim on any OS (no Jetson, CUDA, ZED SDK, ROS2)
2. **Dev Docker image** — containerized sim for CI and isolated testing

## Requirements

- [Git](https://git-scm.com/) and [Pixi](https://pixi.sh) (for the native path)
- [Docker](https://docker.com) (for the container path)
- Python 3.10+ (only needed if you skip Pixi)

## Pixi (recommended)

[Pixi](https://pixi.sh) provides a reproducible cross-platform dev environment:

```bash
# Clone the repo
git clone https://github.com/McGill-Aerial-Design/NOMAD.git
cd NOMAD

# Pixi automatically creates the environment from pixi.toml
pixi run dev          # run Edge Core in sim mode on http://localhost:8000
pixi run test         # pytest
pixi run test-api     # exercise every REST endpoint against a running server
pixi run lint         # ruff check
pixi run fmt          # ruff format
pixi run docs         # serve this documentation site
pixi run docs-build   # build the docs for deployment
```

The `pixi run dev` command starts Edge Core with the `--sim --no-vision` flags so
it runs without any hardware. Open `http://localhost:8000/docs` for the interactive
Swagger UI.

### Git hooks

Install pre-commit hooks once so lint/format run automatically before every commit:

```bash
pixi run pre-commit install
```

## Dev Docker

A lightweight x86 Docker image is provided for CI and isolated testing. The
easiest path is the pixi wrappers, which bring up Edge Core **and** ArduPilot
SITL (Copter 4.6.3) with telemetry wired in — no GPU needed:

```bash
pixi run dev-up       # build + start Edge Core + SITL (detached)
pixi run dev-ps       # status
pixi run test-api     # smoke-test every endpoint
pixi run dev-logs     # follow logs
pixi run dev-down     # stop + remove
```

Edge Core is on `:8000`, MAVLink SITL on TCP `:5760` (connect Mission Planner
there). The dev stack ships a committed dev API key (`nomad-dev-key`) so it works
out of the box; production sets its own key locally.

> SITL 4.6.3 is built once from source:
> `docker build -t nomad-sitl:copter-4.6.3 --build-arg COPTER_TAG=Copter-4.6.3 https://github.com/radarku/ardupilot-sitl-docker.git`

## Configuration profiles

Switch the whole environment (sim / drone / minimal dev) with one command. The
loader updates `config/nomad.env` **and** the Mission Planner plugin config
(API key, endpoint, active-profile indicator):

```bash
pixi run profile-list        # list profiles
pixi run profile-load dev    # or: sim, drone
pixi run profile-show        # show the active profile
```

VS Code users get the same via **Run Task → "Profile: …"**. See
[Configuration](configuration.md#profiles) for details.

## What next

- [Architecture overview](architecture.md) — understand the system design
- [Configuration reference](configuration.md) — configure your environment
- [Writing a module](writing_a_module.md) — extend NOMAD with custom capabilities
