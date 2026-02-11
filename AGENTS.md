# NOMAD Project Quick Reference for AI Agents

This document provides essential information for AI coding assistants working on the NOMAD project. It contains quick references that avoid repeated discovery of project-specific details.

---

## 1. Connection Details

### Jetson Orin Nano (Drone)

| Property | Value |
|----------|-------|
| **Tailscale IP** | `100.85.121.98` |
| **Tailscale Hostname** | `ubuntu` |
| **SSH User** | `mad` |
| **SSH Command** | `ssh mad@100.85.121.98` |
| **Home Directory** | `/home/mad/NOMAD/` |
| **Edge Core Port** | `8000` |
| **API Docs URL** | `http://100.85.121.98:8000/docs` |

### Ground Station (Windows)

| Property | Value |
|----------|-------|
| **Tailscale IP** | `100.76.127.17` |
| **Hostname** | `nomad-groundstation-windows` |

### Quick Connectivity Check

```bash
# Test Tailscale is working
tailscale status

# SSH to Jetson
ssh mad@100.85.121.98

# Test Edge Core API from Windows PowerShell
Invoke-WebRequest -Uri 'http://100.85.121.98:8000/' -UseBasicParsing
```

---

## 2. Project Structure

```
NOMAD/
|-- edge_core/              # Python FastAPI server (runs on Jetson)
|   |-- api.py              # API endpoints
|   |-- main.py             # Entry point
|   |-- health_monitor.py   # Jetson hardware monitoring
|   |-- nav_controller.py   # Navigation commands
|   |-- vio_pipeline.py     # Visual-Inertial Odometry
|   |-- ros_*               # ROS2 bridge modules
|
|-- mission_planner/        # C# plugin (runs on Windows GCS)
|   |-- src/
|       |-- EnhancedHealthDashboard.cs   # Health + network display
|       |-- JetsonConnectionManager.cs   # API client
|       |-- EmbeddedVideoPlayer.cs       # RTSP video
|       |-- LinkHealthPanel.cs           # MAVLink dual-link
|       |-- NOMADPlugin.cs               # Plugin entry point
|
|-- scripts/
|   |-- build/              # Build scripts
|   |   |-- build_plugin_windows.ps1     # C# plugin build
|   |   |-- build_nvblox_msgs.sh         # nvblox message package build
|   |   |-- install_vpi_container.sh     # VPI dependency install
|   |-- run/                # Runtime/startup scripts
|   |   |-- start_nomad_full.sh          # Full system startup (main entry point)
|   |   |-- start_isaac_ros_auto.sh      # Isaac ROS container lifecycle
|   |   |-- restart_nomad.sh             # Kill-all and restart
|   |   |-- launch_nvblox_performance.sh # Performance-tuned nvblox launch
|   |-- setup/              # One-time setup scripts
|   |   |-- setup_jetson.sh              # Full Jetson initial setup
|   |   |-- setup_service.sh             # systemd nomad.service install
|   |   |-- setup_ssh_jetson.ps1         # SSH key setup (Windows)
|   |   |-- setup_jetson_remote.py       # Remote Jetson setup via SSH
|   |   |-- fix_power_mode_25w_v2.sh     # Jetson 25W power mode config
|   |-- dev/                # Development tools
|   |   |-- run_dev.ps1                  # Windows sim mode
|   |   |-- run_dev.sh                   # Linux/macOS sim mode
|   |-- task1/              # Task 1 AI processing
|   |   |-- process_task1_ai.py          # Multi-provider AI image analysis
|   |-- hardware/           # Hardware test utilities
|       |-- servo_test.c                 # GPIO servo test (C)
|       |-- sw_servo_test.py             # Servo sweep test (Python)
|
|-- tailscale/              # VPN configuration and managers
|   |-- src/
|       |-- tailscale_manager.py         # Tailscale monitoring
|       |-- network_monitor.py           # 4G/LTE monitoring
|
|-- config/
|   |-- env/jetson.env      # Environment variables (IPs, paths)
|   |-- video_streams.json  # MediaMTX stream configuration
```

---

## 3. Key Configuration File

**Location**: `config/env/jetson.env`

This file contains:
- Jetson Tailscale IP (`TAILSCALE_IP=100.85.121.98`)
- Ground Station IP (`GCS_IP=100.76.127.17`)
- Home paths (`/home/mad/NOMAD/`)
- Port configuration
- Feature flags

Always check this file for actual IP addresses and paths.

---

## 4. Edge Core API Endpoints

### System
- `GET /` - Service info
- `GET /health` - Basic health check
- `GET /health/detailed` - Full Jetson metrics
- `GET /status` - Current system state

### Network (Added in PR #3)
- `GET /network/status` - Tailscale + modem + reachability
- `POST /network/reconnect` - Trigger Tailscale reconnect
- `GET /network/ping/{host}` - Ping utility

### Task 1 (Outdoor GPS)
- `POST /api/task/1/capture` - Capture GPS snapshot

### Task 2 (Indoor VIO)
- `POST /api/task/2/reset_map` - Clear exclusion map
- `POST /api/task/2/target_hit` - Register target position
- `GET /api/task/2/exclusion_map` - Get hit targets

### VIO
- `GET /api/vio/status` - Pipeline health
- `GET /api/vio/pose` - Current position/orientation
- `GET /api/vio/trajectory` - Path history
- `DELETE /api/vio/trajectory` - Clear trajectory
- `POST /api/vio/reset_origin` - Reset tracking origin

### Navigation
- `GET /api/nav/status` - Navigation controller status
- `POST /api/nav/velocity` - Send velocity command
- `POST /api/nav/position` - Send position target
- `POST /api/nav/stop` - Emergency stop

### Isaac ROS (Task 2)
- `GET /api/isaac/status` - Isaac ROS bridge status
- `POST /api/isaac/start` - Start Isaac ROS container
- `POST /api/isaac/stop` - Stop Isaac ROS container

### Servo / Nozzle Control
- `POST /api/servo/camera/tilt?angle={0-180}` - Set nozzle servo angle
- `POST /api/servo/shooter/trigger?duration_ms={ms}` - Trigger water shooter GPIO
- `GET /api/servo/status` - Get servo and GPIO output status
- ROS topic: `/nomad/servo/nozzle_angle` (Float32) - Autonomous servo control via ros_http_bridge

---

## 5. Development Workflows

### Pull Latest Code to Jetson

```bash
ssh mad@100.85.121.98
cd ~/NOMAD
git stash          # Save local changes
git pull origin main
git stash pop      # Restore local changes (optional)
```

### Start Edge Core Manually

```bash
ssh mad@100.85.121.98
cd ~/NOMAD
python3 -m edge_core.main --port 8000
```

### Build Mission Planner Plugin (Windows)

```powershell
cd NOMAD
.\scripts\build\build_plugin_windows.ps1
```

Output: `C:\Users\<user>\AppData\Local\Mission Planner\plugins\NOMADPlugin.dll`

### Test API Endpoints (Windows PowerShell)

```powershell
# Basic health
Invoke-WebRequest -Uri 'http://100.85.121.98:8000/health' -UseBasicParsing

# Network status (JSON pretty print)
(Invoke-WebRequest -Uri 'http://100.85.121.98:8000/network/status' -UseBasicParsing).Content | ConvertFrom-Json | ConvertTo-Json -Depth 5

# Ping test
Invoke-WebRequest -Uri 'http://100.85.121.98:8000/network/ping/8.8.8.8' -UseBasicParsing
```

---

## 6. Common Issues and Solutions

### Issue: SSH prompts for password
**Solution**: Use correct username `mad`, not `nomad`
```bash
ssh mad@100.85.121.98
```

### Issue: Jetson IP not responding
**Check Tailscale status**:
```powershell
tailscale status
```
The Jetson shows as `ubuntu` not `nomad-jetson`.

### Issue: Edge Core not running
**Start manually**:
```bash
ssh mad@100.85.121.98 "cd ~/NOMAD && python3 -m edge_core.main --port 8000 &"
```

### Issue: Network endpoint returns null modem
**Expected**: Modem will be null if no 4G/LTE USB modem is connected.

### Issue: Code on Jetson is outdated
**Deploy latest**:
```bash
ssh mad@100.85.121.98
cd ~/NOMAD
git fetch origin
git pull origin main
```

---

## 7. Important Git Commits

| Commit | Description |
|--------|-------------|
| `0cbfce5` | Network status API endpoints (PR #3, Ryan Plouffe) |
| `86c7f8b` | Merge PR #3 feature/network-endpoints |
| `7a6a3e6` | Add Notification Panel and Service |

To check which commit is deployed on Jetson:
```bash
ssh mad@100.85.121.98 "cd ~/NOMAD && git log --oneline -1"
```

---

## 8. Ports Reference

| Service | Port | Protocol | Location |
|---------|------|----------|----------|
| Edge Core API | 8000 | TCP | Jetson |
| MAVLink Telemetry | 14550 | UDP | Jetson -> GCS |
| RTSP Video | 8554 | TCP | Jetson (MediaMTX) |
| SSH | 22 | TCP | Jetson |

---

## 9. Tailscale Network

The Jetson connects via Tailscale VPN for:
- Remote SSH access
- API access to Edge Core
- MAVLink telemetry forwarding
- RTSP video streaming

**Configuration**: `tailscale/SETUP.md`

---

## 10. Documentation Index

| Topic | File |
|-------|------|
| Project Overview | `README.md` |
| Architecture | `docs/architecture.md` |
| Operations Runbook | `docs/OPERATIONS_RUNBOOK.md` |
| Jetson Deployment | `docs/JETSON_DEPLOYMENT.md` |
| Tailscale Setup | `tailscale/SETUP.md` |
| Video Streaming | `docs/VIDEO_STREAMING.md` |
| Navigation Architecture | `docs/JETSON_NAV_ARCHITECTURE.md` |
| Isaac ROS + nvblox Setup | `docs/ISAAC_ROS_NVBLOX_SETUP.md` |
| Servo Control | `docs/SERVO_CONTROL.md` |
| This Quick Reference | `AGENTS.md` |

---

## 11. Common AI Agent Mistakes (Learn from This)

### TaskSync Protocol Violations

**Mistake**: Stopping the conversation after asking clarification questions instead of continuing the task cycle.

**Example of Wrong Behavior**:
```
Agent: "I have a few questions before implementing..."
User: [Answers questions]
Agent: [Session ends or waits indefinitely]
```

**Correct TaskSync Behavior**:
```
Agent: [Asks questions via py -c "question = input('...')"]
User: [Answers via terminal]
Agent: [IMMEDIATELY continues with implementation]
Agent: [Completes task]
Agent: [IMMEDIATELY requests next task via py -c "task = input('')"]
```

**Key Rule**: NEVER stop the conversation after asking questions. The TaskSync protocol requires continuous operation until the user explicitly terminates with "stop", "end", "terminate", or "quit".

**Prevention**: Always use the terminal command `py -c "question = input('...')"` for questions and immediately continue with the task after receiving answers.

### TaskSync Does NOT Apply to Subagents

The TaskSync protocol (terminal-based task input loop, continuous operation, no session termination) applies **only to the primary agent** running in the main chat. **Subagents** spawned via `runSubagent` are stateless, single-task workers. They should:
- Complete their assigned task and return results
- NOT attempt to run `py -c "task = input('')"` or any TaskSync terminal commands
- NOT attempt continuous operation or task loops
- Simply report findings/results back to the primary agent

---

*Last Updated: February 5, 2026*
