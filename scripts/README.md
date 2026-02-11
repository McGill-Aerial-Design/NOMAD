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
| `build_nvblox_msgs.sh` | Build nvblox_msgs package inside Isaac ROS container |
| `install_vpi_container.sh` | Install VPI dev libraries inside Isaac ROS container |

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
| `setup_jetson_remote.py` | Remote Jetson setup via SSH (alternative to on-device setup) |
| `fix_power_mode_25w_v2.sh` | Add 25W MAXN power mode to Jetson Orin Nano |

## dev/

| Script | Description |
|--------|-------------|
| `run_dev.ps1` | Run Edge Core in simulation mode on Windows |
| `run_dev.sh` | Run Edge Core in simulation mode on Linux/macOS |

```bash
# Windows
.\scripts\dev\run_dev.ps1

# Linux/macOS
./scripts/dev/run_dev.sh
```

Sets `NOMAD_SIM_MODE=true` for mock hardware. API at `http://localhost:8000/docs`.

## task1/

| Script | Description |
|--------|-------------|
| `process_task1_ai.py` | Multi-provider AI image analysis (Gemini, Ollama, OpenRouter) |
| `README_AI.md` | Full documentation for Task 1 AI processing |

```powershell
python scripts\task1\process_task1_ai.py --provider gemini --gemini-key YOUR_KEY
```

## hardware/

| Script | Description |
|--------|-------------|
| `servo_test.c` | Low-level GPIO servo test (C, uses gpiochip0) |
| `sw_servo_test.py` | Servo sweep test wrapper (compiles and runs servo_test.c) |
