# Scripts

Organized into subfolders by purpose.

## Folder Layout

```
scripts/
  build/        Build and compilation scripts
  run/          Runtime and startup scripts
  setup/        One-time setup and provisioning
  dev/          Local development tools
  task1/        Task 1 AI image processing
  hardware/     Hardware test utilities
```

## build/

| Script | Description |
|--------|-------------|
| `build_plugin_windows.ps1` | Build the Mission Planner C# plugin on Windows |

## run/

| Script | Description |
|--------|-------------|
| `start_nomad_full.sh` | Full system startup (Edge Core, MAVLink, MediaMTX, Isaac ROS) |
| `start_isaac_ros_auto.sh` | Isaac ROS container lifecycle (start/stop/restart/status/logs/shell) |
| `restart_nomad.sh` | Kill all NOMAD processes and restart everything |
| `launch_nvblox_performance.sh` | Launch nvblox with memory-optimized config for Orin Nano |

## setup/

| Script | Description |
|--------|-------------|
| `setup_jetson.sh` | Full Jetson initial setup (deps, Tailscale, venv, MAVLink, firewall) |
| `setup_service.sh` | Install systemd `nomad.service` for Edge Core |
| `setup_ssh_jetson.ps1` | Set up passwordless SSH from Windows to Jetson |
| `fix_power_mode_25w_v2.sh` | Add 25W MAXN power mode to Jetson Orin Nano |

## dev/

| Script | Description |
|--------|-------------|
| `run_dev.ps1` | Run Edge Core in simulation mode on Windows |
| `run_dev.sh` | Run Edge Core in simulation mode on Linux/macOS |

The `dev/` directory also contains 50+ diagnostic and test scripts for ZED camera, USB, ROS, nvblox, and mesh debugging. Key ones include:

| Script | Description |
|--------|-------------|
| `check_nvblox.sh` | Check nvblox node status |
| `check_mesh_topics.sh` | Check mesh topic publishing |
| `diag_zed.sh` | ZED camera diagnostics |
| `diag_zed_usb.sh` | ZED USB connection diagnostics |
| `full_restart_ros.sh` | Full restart of ROS components |
| `restart_nvblox.sh` | Restart nvblox node |
| `restart_bridge.sh` | Restart the ros_http_bridge |
| `ros_full_launch.sh` | Full ROS launch for development |

```bash
# Windows
.\scripts\dev\run_dev.ps1

# Linux/macOS
./scripts/dev/run_dev.sh
```

Sets `NOMAD_SIM_MODE=true` for mock hardware. API at `http://localhost:8000/docs`.

## hardware/

| Script | Description |
|--------|-------------|
| `servo_test.c` | Low-level GPIO servo test (C, uses gpiochip0) |
| `sw_servo_test.py` | Servo sweep test wrapper (compiles and runs servo_test.c) |
