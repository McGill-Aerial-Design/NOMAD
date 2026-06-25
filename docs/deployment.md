# Deployment

NOMAD runs on an onboard companion computer (typically an NVIDIA Jetson Orin Nano)
with a ground station running Mission Planner + the NOMAD plugin.

## Deployment options

| Method | Use case |
|--------|----------|
| **Jetson Isaac ROS image** | Production perception/odometry container (ZED + ROS-HTTP bridge) on the Jetson |
| **Bare-metal systemd** | Edge Core API, MediaMTX, and mavlink-router as host services |

## Prerequisites (Jetson)

- NVIDIA Jetson Orin Nano (or other aarch64 platform)
- JetPack 6.2 (L4T r36.4)
- [Docker](https://docs.docker.com/engine/install/) with `nvidia-container-toolkit`
- [Tailscale](https://tailscale.com) for VPN connectivity
- 4G/LTE modem (optional, for remote connectivity)
- ZED 2i camera

## Jetson Isaac ROS image

> **Testing without hardware?** See [Simulation Environments](simulation.md) for
> the CPU-only ROS2 bridge image (`Dockerfile.sim-ros`, runs in CI), the
> ArduPilot + Gazebo Harmonic image (`Dockerfile.sim-gazebo`), and the full
> Isaac ROS + ZED SDK image for x86_64 GPU machines (`Dockerfile.sim-isaac`).

[docker/Dockerfile.jetson](../docker/Dockerfile.jetson) builds the on-board
perception/odometry container, layered on the official **Isaac ROS dev base
image** (which must be built on the Jetson first — see the Isaac ROS docs and
`isaac_ros_common`). It provides:

- the ZED SDK + ZED 2i camera wrapper runtime
- `isaac_ros_nvblox_utils` (optional — drop the nvblox stage if you don't use it)
- GStreamer for the RTSP video bridge
- the NOMAD ROS-HTTP bridge (`edge_core/ros_http_bridge`), copied onto
  `PYTHONPATH` so it is self-contained — no workspace mount needed

The Edge Core API, MediaMTX (RTSP), and mavlink-router run alongside it as host
services — see [Bare-metal systemd](#bare-metal-systemd).

### Build

```bash
docker build -f docker/Dockerfile.jetson \
  --build-arg BASE_IMAGE=isaac_ros_dev-aarch64 \
  -t nomad-jetson:latest .
```

Build on the Jetson (or a self-hosted aarch64 runner): the image targets
aarch64, downloads the ZED SDK, and layers on the GPU ROS stack, so it cannot
build on generic x86 CI runners.

### Run

```bash
docker run --rm --runtime nvidia --privileged --network host \
  -v /dev:/dev \
  -v /usr/local/zed:/usr/local/zed \
  -v /run/udev:/run/udev \
  -v /tmp/argus_socket:/tmp/argus_socket \
  nomad-jetson:latest \
  python3 -m edge_core.ros_http_bridge.main --host 127.0.0.1
```

The default `CMD` is an interactive shell (the entrypoint sources ROS2 first), so
omit the trailing command to drop into the container for the ZED wrapper / nvblox
launch files.

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
