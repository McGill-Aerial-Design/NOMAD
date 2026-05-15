# Scripts

NOMAD runtime is structured around **one CLI** (`scripts/nomad`) that
dispatches to **one script per service** (`scripts/services/*.sh`). Each
service script implements `start | stop | restart | status | logs` and owns
*only* its own processes — no cross-service pkills.

## Layout

```
scripts/
  nomad                CLI dispatcher — the single entry point
  lib/common.sh        Shared helpers (logging, env loader, kill helpers)
  services/            One script per service (8 total)
  setup/               One-time provisioning (run once on a fresh Jetson)
  build/               Build / compile helpers
  dev/                 Local development + ad-hoc diagnostics
  hardware/            Low-level hardware test utilities
```

## CLI quickstart

```bash
nomad list                    # show all services and their autostart flags
nomad start all               # start the autostart set (see config/nomad.env)
nomad stop all
nomad status                  # status of all services
nomad start video_bridge      # start a single service
nomad restart ros_http_bridge
nomad logs zed_wrapper        # tail logs for one service
```

## Services

The 8 services, in start order. **`all` covers every service whose
`NOMAD_AUTOSTART_*` flag in `config/nomad.env` is true.** By default nvblox
is OFF; everything else is ON.

| Service               | Where               | Owns                                                |
|-----------------------|---------------------|-----------------------------------------------------|
| `edge_core`           | host                | `edge_core.main` FastAPI process                    |
| `mavlink_router`      | host                | `mavlink-routerd`                                   |
| `mediamtx`            | host                | `mediamtx` RTSP server                              |
| `isaac_ros_container` | host (Docker)       | the `nomad_isaac_ros` container (`sleep infinity`)  |
| `zed_wrapper`         | in container        | `ros2 launch zed_wrapper zed_camera.launch.py` + helper nodes |
| `ros_http_bridge`     | in container        | `ros_http_bridge.py` (with restart loop)            |
| `video_bridge`        | in container (via API) | `simple_video_bridge.py` via Edge Core's `/api/video/start` |
| `nvblox`              | in container        | `nomad_zed_nvblox.launch.py` — **opt-in only**      |

Services can be started/stopped **independently**. Stopping `nvblox` doesn't
touch ZED; stopping `zed_wrapper` doesn't touch the container; stopping
`isaac_ros_container` does cascade (everything inside it dies with it).

## One-time setup

```bash
sudo bash infra/systemd/install.sh          # install per-service systemd units
bash scripts/setup/provision_isaac_ros.sh   # clone ZED wrapper, install SDK, build packages
```

After provisioning, the container can be torn down and recreated without
re-running provisioning (the SDK install and apt deps are persisted into the
image via `docker commit`).

## Configuration

**Everything** lives in `config/nomad.env`. There is no `jetson.env` and no
inline `export` statements in scripts. Edit `config/nomad.env`, then:

```bash
nomad restart all                           # apply changes
# or, if running under systemd:
sudo systemctl restart nomad.target
```

To toggle autostart for a service (e.g. enable nvblox at boot), edit the
corresponding `NOMAD_AUTOSTART_*` flag, then re-run `sudo bash
infra/systemd/install.sh` to reconcile the enabled set.

## setup/

| Script                     | Description                                              |
|----------------------------|----------------------------------------------------------|
| `provision_isaac_ros.sh`   | One-time: clone ZED wrapper, install ZED SDK + apt deps in container, build packages, commit image |
| `setup_jetson.sh`          | Full Jetson initial setup (deps, Tailscale, venv, MAVLink, firewall) |
| `setup_service.sh`         | (legacy) shim — prefer `infra/systemd/install.sh`        |
| `setup_ssh_jetson.ps1`     | Passwordless SSH from Windows to Jetson                  |
| `fix_power_mode_25w_v2.sh` | Add 25W MAXN power mode                                  |

## dev/

Diagnostics, USB / ZED probes, and Windows simulation entry points
(`run_dev.ps1`, `run_dev.sh`). The historic `restart_nvblox.sh`,
`restart_bridge.sh`, `full_restart_ros.sh`, `ros_full_launch.sh` are gone —
their behavior is now `nomad restart nvblox`, `nomad restart ros_http_bridge`,
etc.

## build/ and hardware/

Unchanged. See the files in those directories.
