# Deployment

NOMAD runs on an onboard companion computer (typically an NVIDIA Jetson Orin Nano)
with a ground station running Mission Planner + the NOMAD plugin.

## Deployment options

| Method | Use case |
|--------|----------|
| **Jetson all-in-one Docker image** | Production — single container runs all services |
| **Bare-metal systemd** | Developers who prefer process-level management |

## Prerequisites (Jetson)

- NVIDIA Jetson Orin Nano (or other aarch64 platform)
- JetPack 6.0+ (L4T r36.4+)
- [Docker](https://docs.docker.com/engine/install/) with `nvidia-container-toolkit`
- [Tailscale](https://tailscale.com) for VPN connectivity
- 4G/LTE modem (optional, for remote connectivity)
- ZED 2i camera

## Jetson all-in-one image

The Docker image bundles all services:

- Edge Core (Python FastAPI)
- MediaMTX (RTSP server)
- mavlink-router
- Isaac ROS container (ZED wrapper, nvblox, ROS-HTTP bridge)

### Build

```bash
docker build -f docker/Dockerfile.jetson -t nomad-jetson:latest .
```

Build requires a Jetson device with the Isaac ROS dev base image. The image is
too large to build on generic CI runners — use a self-hosted aarch64 runner or
build directly on the Jetson.

### Run

```bash
docker run --rm --runtime nvidia --privileged --network host \
  -v /dev:/dev \
  -v /usr/local/zed:/usr/local/zed \
  -v /run/udev:/run/udev \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -v $(pwd)/config:/config \
  -e NOMAD_API_KEY="<your-api-key>" \
  nomad-jetson:latest
```

### Configuration

Mount your `config/nomad.env` into `/config/` or pass individual environment
variables with `-e`. See [Configuration](configuration.md) for the full reference.

## Bare-metal systemd

Systemd units are provided in `infra/systemd/`. Each service has its own unit file
and a shared `install.sh` script fills in paths from `config/nomad.env`.

```bash
# Install all units
cd NOMAD
sudo ./infra/systemd/install.sh

# Control services
nomad start all       # start all autostart-enabled services
nomad stop all        # stop all services
nomad restart edge    # restart just Edge Core
nomad status          # show service statuses
```

## Ground station

1. Install [Mission Planner](https://ardupilot.org/planner/)
2. Install the NOMAD plugin (see `mission_planner/README.md`)
3. Configure the Jetson IP, API key, and RTSP URLs in the plugin settings
4. Connect to the drone via ELRS and/or Tailscale

## Ports

| Service | Port | Protocol | Location |
|---------|------|----------|----------|
| Edge Core API | 8000 | TCP | companion |
| MAVLink LTE/Tailscale | 14560 | UDP | companion → GCS |
| MAVLink RadioMaster | 14550 | UDP | GCS local |
| MAVLink Plugin Router | 14600 | UDP | Mission Planner |
| RTSP Video | 8554 | TCP | companion |
| SSH | 22 | TCP | companion |
