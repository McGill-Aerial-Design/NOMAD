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
|   |-- api.py              # REST / WebSocket API endpoints (~125 endpoints across all tags)
|   |-- main.py             # Entry point, service orchestration
|   |-- state.py            # Global state manager (singleton)
|   |-- health_monitor.py   # Jetson hardware monitoring
|   |-- mavlink_interface.py # MAVLink telemetry + command interface
|   |-- nav_controller.py   # Velocity / position command routing
|   |-- servo_controller.py # Camera tilt / water shooter PWM
|   |-- rc_servo_bridge.py # RC channel -> servo angle bridge
|   |-- spray_controller.py # Fire-extinguisher spray control
|   |-- operational_mode.py # Operational mode state machine
|   |-- video_stream_manager.py # Video bridge / overlay / source switching
|   |-- isaac_ros_bridge.py # Isaac ROS / nvblox container lifecycle
|   |-- ros_http_bridge.py # ROS <-> HTTP bridge
|   |-- gdrive_upload.py    # Google Drive upload helpers
|   |-- ipc.py              # ZMQ/IPC for high-rate ROS data
|   |-- logging_service.py  # Structured logging utilities
|   |-- time_manager.py     # Time synchronization
|   |-- geospatial.py       # GPS / ENU / NED conversions
|   |-- models.py           # Pydantic data models
|   |-- target_localizer/   # HSV circle detection, building model, descriptions
|   |-- ros/                # ROS2 utility nodes (VIO, video, obstacles)
|
|-- mission_planner/        # C# plugin (runs on Windows GCS)
|   |-- src/
|       |-- NOMADPlugin.cs               # Plugin entry point
|       |-- NOMADMainScreen.cs           # Main plugin host screen
|       |-- NOMADDashboardView.cs        # System overview dashboard
|       |-- NOMADTask1View.cs            # Task 1 capture / submit UI
|       |-- NOMADTask2View.cs            # Task 2 VIO / WASD UI
|       |-- NOMADVideoView.cs            # Standalone video panel
|       |-- NOMADHealthView.cs           # Health metrics view
|       |-- NOMADLinksView.cs            # Link status view
|       |-- NOMADBoundaryView.cs         # Geofence / boundary display
|       |-- NOMADTerminalView.cs         # Remote terminal tab
|       |-- NOMADSettingsForm.cs         # Plugin settings
|       |-- ServiceControlPanel.cs       # Service status / control panel
|       |-- EnhancedHealthDashboard.cs     # Health + network display
|       |-- JetsonConnectionManager.cs   # API client
|       |-- EmbeddedVideoPlayer.cs       # RTSP video
|       |-- LinkHealthPanel.cs           # MAVLink dual-link
|       |-- JetsonHealthTab.cs           # Per-tab health display
|       |-- JetsonTerminalControl.cs     # Remote terminal access
|       |-- SLAM3DView.cs + SLAM3D/      # 3D nvblox mesh viewer
|       |-- Task1UploadPanel.cs          # Target grid + Google Drive upload
|       |-- PayloadControlPanel.cs       # Payload drop / water shooter
|       |-- GoogleDriveUploadService.cs  # GDrive upload service
|       |-- AIDescriptionService.cs      # AI description generation
|       |-- DualLinkSender.cs            # HTTP/MAVLink dual-path sender
|       |-- NotificationPanel.cs + NotificationService.cs # Toast notification system
|       |-- MAVLinkConnectionManager.cs  # MAVLink telemetry mgmt
|       |-- TelemetryInjector.cs         # Telemetry / HUD injection
|       |-- EkfSourceControlPanel.cs     # EKF source switching
|       |-- EnhancedWASDControl.cs        # WASD nav controls
|       |-- SnapshotManager.cs           # ZED snapshot mgmt
|       |-- BuildingViewer3D.cs          # 3D building viewer
|       |-- ZedCalibrationView.cs        # ZED calibration UI
|       |-- FoxglovePanel.cs             # Foxglove integration
|       |-- Rviz2View.cs                 # RViz2 integration
|       |-- MapOverlayManager.cs         # Map overlay manager
|       |-- BoundaryManager.cs           # Geofence boundary manager
|       |-- MissionConfig.cs + NOMADConfig.cs # Configuration models
|       |-- NOMADTheme.cs                # UI theming
|       |-- NOMADViewBase.cs             # Base view class
|
|-- scripts/
|   |-- nomad                            # Single CLI dispatcher (start|stop|restart|status|logs)
|   |-- lib/common.sh                    # Shared service-script helpers
|   |-- services/                        # One script per service (8 total)
|   |   |-- edge_core.sh                 # Host: Edge Core API
|   |   |-- mavlink_router.sh            # Host: MAVLink Router
|   |   |-- mediamtx.sh                  # Host: RTSP server
|   |   |-- isaac_ros_container.sh       # Host: Isaac ROS Docker container (sleep infinity)
|   |   |-- zed_wrapper.sh               # In container: ZED ROS2 wrapper + helper nodes
|   |   |-- ros_http_bridge.sh           # In container: ros_http_bridge.py
|   |   |-- video_bridge.sh              # In container (via API): simple_video_bridge.py
|   |   |-- nvblox.sh                    # In container: nvblox (OPT-IN, not autostarted)
|   |-- build/                           # Build / compile helpers
|   |-- setup/
|   |   |-- provision_isaac_ros.sh       # One-time: ZED SDK install + colcon build
|   |   |-- setup_jetson.sh              # Jetson initial setup
|   |   |-- setup_ssh_jetson.ps1         # SSH key setup (Windows)
|   |   |-- fix_power_mode_25w_v2.sh     # 25W MAXN power mode
|   |-- dev/                             # Development tools + ad-hoc diagnostics
|   |   |-- run_dev.ps1                  # Windows sim mode
|   |   |-- run_dev.sh                   # Linux/macOS sim mode
|   |-- hardware/                        # Hardware test utilities
|       |-- servo_test.c                 # GPIO servo test (C)
|       |-- sw_servo_test.py             # Servo sweep test (Python)
|
|-- infra/systemd/                       # One systemd unit per service + install.sh
|   |-- nomad.target                     # Pulls in the autostart set
|   |-- nomad-edge-core.service
|   |-- nomad-mavlink-router.service
|   |-- nomad-mediamtx.service
|   |-- nomad-isaac-ros-container.service
|   |-- nomad-zed-wrapper.service
|   |-- nomad-ros-http-bridge.service
|   |-- nomad-video-bridge.service
|   |-- nomad-nvblox.service             # opt-in
|   |-- install.sh                       # Install + reconcile from config/nomad.env
|
|-- tailscale/                           # VPN configuration and managers
|   |-- src/
|       |-- tailscale_manager.py         # Tailscale monitoring
|       |-- network_monitor.py           # 4G/LTE monitoring
|
|-- config/
|   |-- nomad.env                        # SINGLE SOURCE OF TRUTH for runtime config
|   |-- video_streams.json               # MediaMTX stream configuration
```

---

## 3. Key Configuration File

**Location**: `config/nomad.env`

This is the **only** runtime config file. Every script and the Edge Core
systemd unit reads it via `EnvironmentFile=`. It contains:

- Service autostart flags (`NOMAD_AUTOSTART_*` — nvblox defaults to false)
- Jetson / GCS IPs, ports, auth tokens
- Per-service tuning (ZED resolution, ROS bridge rate, nvblox profile path)

Edit it in place; then `nomad restart all` (or `sudo systemctl restart nomad.target`).

---

## 4. Edge Core API Endpoints

> **Full codegen**: The API is defined in `edge_core/api.py` (8K+ lines). Only key endpoints are listed below. Always verify the source for the latest parameter list and behaviour.

### System
- `GET /` - Service info
- `GET /health` - Basic health check
- `GET /health/detailed` - Full Jetson metrics (CPU, GPU, temp, memory, disk)
- `GET /status` - Current system state
- `GET /api/services/status` - systemd / process status for nomad, mediamtx, mavlink-routerd

### Network
- `GET /network/status` - Tailscale + modem + reachability
- `POST /network/reconnect` - Trigger Tailscale reconnect
- `GET /network/ping/{host}` - Ping utility

### Task 1 (Outdoor Reconnaissance)
- `POST /api/task/1/target/capture` - Trigger target detection + description (primary)
- `POST /api/task/1/target/save` - Save targets to competition .txt file
- `POST /api/task/1/target/clear` - Clear all captured targets
- `POST /api/task/1/target/ground_alt` - Set global ground altitude for 3D back-projection
- `POST /api/task/1/target/{target_id}/plane_override` - Override wall/ground/roof classification for a target
- `GET /api/task/1/target/detections` - Get current frame detection overlay data
- `GET /api/task/1/target/list` - List captured targets
- `GET /api/task/1/target/list_structured` - Structured target list (with metadata)
- `GET /api/task/1/target/model` - Print building model summary
- `POST /api/task/1/building/corner` - Save one building corner GPS (on-site calibration)
- `GET /api/task/1/building/corners` - List saved building corners
- `DELETE /api/task/1/building/corners` - Clear all building corners
- `POST /api/task/1/building/height` - Update building height
- `POST /api/task/1/building/wall/override` - Override face wall directions
- `POST /api/task/1/building/corners/apply` - Apply saved corners to target_localizer (rebuild building model)
- `POST /api/task/1/capture` - Legacy capture endpoint
- `GET /api/task/1/captures` - List capture folders
- `GET /api/task/1/images/{filename}` - Legacy image download
- `GET /api/task/1/images/{folder}/{filename}` - Download captured image/metadata
- **Full guide**: `docs/TASK1_COMPETITION_GUIDE.md`

### Task 2 (Indoor VIO)
- `POST /api/task/2/reset_map` - Clear exclusion map
- `POST /api/task/2/target_hit` - Register target position
- `GET /api/task/2/exclusion_map` - Get hit targets

### VIO
- `GET /api/vio/status` - Pipeline health
- `POST /api/vio/update` - Push external VIO pose update (internal bridge)
- `GET /api/vio/pose` - Current position/orientation
- `GET /api/vio/trajectory` - Path history
- `DELETE /api/vio/trajectory` - Clear trajectory
- `POST /api/vio/reset_origin` - Reset tracking origin
- `GET /api/vio/calibration` - ZED calibration state
- `POST /api/vio/area/save` - Save VIO relocalization area map
- `POST /api/vio/area/load` - Load VIO relocalization area map
- `POST /api/vio/area/relocalize` - Trigger relocalization to saved area

### Navigation
- `GET /api/nav/status` - Navigation controller status
- `POST /api/nav/velocity` - Send velocity command
- `POST /api/nav/position` - Send position target
- `POST /api/nav/stop` - Emergency stop
- `GET /api/nav/enable_guided` - Enable guided mode
- `POST /api/nav2/goal` - Set Nav2 goal for autonomous path
- `GET /api/nav2/status` - Nav2 autonomy status
- `GET /api/nav2/pending` - Pending Nav2 goal
- `POST /api/nav2/feedback` - Nav2 feedback from bridge
- `POST /api/nav2/result` - Nav2 result from bridge
- `GET /api/obstacle_distance` - Last cached obstacle distance
- `POST /api/obstacle_distance` - Update obstacle distance (internal bridge)

### Spray
- `GET /api/spray/status` - Spray controller state
- `POST /api/spray/trigger` - Trigger spray sequence
- `POST /api/spray/abort` - Abort spray sequence

### Isaac ROS (Task 2)
- `GET /api/isaac/status` - Isaac ROS bridge + container status
- `POST /api/isaac/start` - Start Isaac ROS container
- `POST /api/isaac/stop` - Stop Isaac ROS container
- `POST /api/isaac/launch-nvblox` - Launch nvblox with specific config
- `POST /api/isaac/bridge/start` - Start ROS <-> HTTP bridge
- `POST /api/isaac/bridge/stop` - Stop ROS <-> HTTP bridge
- `POST /api/isaac/nvblox/start` - Start nvblox node inside container
- `POST /api/isaac/nvblox/stop` - Stop nvblox node inside container
- `POST /api/isaac/foxglove/start` - Start Foxglove bridge
- `POST /api/isaac/foxglove/stop` - Stop Foxglove bridge
- `GET /api/isaac/foxglove/status` - Foxglove bridge status
- `GET /api/isaac/logs` - Isaac ROS container logs
- `GET /api/isaac/vio` - VIO state from Isaac ROS
- `GET /api/isaac/detections` - Latest detections from Isaac ROS
- `GET /api/isaac/exclusion_map` - Exclusion map from Isaac ROS
- `POST /api/isaac/exclusion_map/add` - Add to Isaac ROS exclusion map
- `POST /api/isaac/exclusion_map/clear` - Clear Isaac ROS exclusion map

### Detections
- `GET /api/detections` - Current frame detections
- `GET /api/detections/status` - Detection pipeline status
- `GET /api/detections/summary` - Summary of persisted detections
- `POST /api/detections/start` - Start detection pipeline
- `POST /api/detections/stop` - Stop detection pipeline
- `POST /api/detections/update` - Push detection update (internal bridge)
- `DELETE /api/detections/history` - Clear persisted detection history

### Video / Streaming
- `GET /api/stream/info` - Stream metadata (bitrate, fps, latency)
- `GET /api/video/status` - Video pipeline status
- `GET /api/video/source` - Current video source
- `POST /api/video/source` - Switch video source
- `POST /api/video/start` - Start video streaming pipeline
- `POST /api/video/stop` - Stop video streaming pipeline
- `POST /api/video/restart` - Restart video streaming pipeline
- `POST /api/video/bridges/start` - Start video bridge(s)
- `GET /api/video/bridges` - List active video bridges
- `GET /api/video/topics` - List available video topics
- `GET /api/video/logs` - Video pipeline logs
- `POST /api/video/overlay/enable` - Enable OSD overlay
- `POST /api/video/overlay/disable` - Disable OSD overlay

### Servo / Nozzle Control
- `GET /api/servo/status` - Get servo and GPIO output status
- `POST /api/servo/enable` - Enable servo output
- `POST /api/servo/disable` - Disable servo output
- `GET /api/servo/camera/tilt` - Get current camera tilt angle
- `POST /api/servo/camera/tilt?angle={0-180}` - Set camera servo angle
- `GET /api/servo/camera/config` - Get servo config (min/max pulse, etc.)
- `POST /api/servo/camera/config` - Update servo config
- `POST /api/servo/shooter/trigger?duration_ms={ms}` - Trigger water shooter GPIO
- `GET /api/servo/rc/status` - RC-to-servo bridge status (channel, last value, angle)
- `POST /api/servo/rc/channel?channel={1-18}` - Change which RC channel controls servo
- ROS topic: `/nomad/servo/nozzle_angle` (Float32) - Autonomous servo control via ros_http_bridge

### Mode
- `GET /api/mode` - Current operational mode
- `POST /api/mode/set` - Set operational mode

### Terminal
- `GET /api/terminal/commands` - List available whitelisted commands
- `GET /api/terminal/logs` - Recent stdout/stderr from executed commands

### WebSockets
- `WS /ws/state` - Real-time system state broadcast (10Hz)
- `WS /ws/slam` - Real-time SLAM / nvblox mesh update stream

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
| MAVLink Local | 14551 | UDP | Jetson (localhost) |
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
| nvblox Visualization | `docs/NVBLOX_VISUALIZATION.md` |
| Mesh Troubleshooting | `docs/TROUBLESHOOTING_MESH.md` |
| Servo Control | `docs/SERVO_CONTROL.md` |
| Object Detection | `docs/OBJECT_DETECTION.md` |
| Dual Link Failover | `docs/DUAL_LINK_FAILOVER.md` |
| Nav2 Integration Plan | `docs/NAV2_INTEGRATION_PLAN.md` |
| Task 2 Manual Positioning | `docs/TASK2_MANUAL_POSITIONING.md` |
| **Task 1 Competition Guide** | **`docs/TASK1_COMPETITION_GUIDE.md`** |
| **TODO / Requirements Audit** | **`todo.md`** |
| Edge Core README | `edge_core/README.md` |
| Target Localizer README | `edge_core/target_localizer/README.md` |
| Mission Planner README | `mission_planner/README.md` |
| Scripts README | `scripts/README.md` |
| This Quick Reference | `AGENTS.md` |

> **IMPORTANT**: Always check `todo.md` before starting work. It contains:
> - A full requirements audit (REQ-1 to REQ-7) with detailed implementation notes
> - Priority-ordered list of missing features (NV-008, operational modes, spray sequence)
> - nvblox config switching notes (requires node restart, 2-3s blind window)
> - Note: `config/nvblox_indoor.yaml` referenced in todo.md does not yet exist

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

### No Fallback Chains -- Pick the Best Approach and Commit

**Rule**: Do NOT implement multi-strategy fallback chains (Strategy 1 -> Strategy 2 -> Strategy 3). Pick the single best approach for a problem and implement it properly. Fallback chains increase code complexity, hide bugs, make debugging harder, and are a sign of lazy engineering.

**Instead of fallbacks**:
- Analyze the problem fully before coding.
- Choose the best solution based on reliability, performance, and simplicity.
- Implement that one solution correctly with proper error handling.
- If the chosen approach has a prerequisite (e.g., a service must be running), ensure the prerequisite is met rather than coding around its absence.

**Bad**: Try ZED SDK, if that fails try RTSP, if that fails try HTTP snapshot, if that fails return empty.
**Good**: Use HTTP snapshot from the video bridge (fastest, most reliable, always available when streaming).

### TaskSync Does NOT Apply to Subagents

The TaskSync protocol (terminal-based task input loop, continuous operation, no session termination) applies **only to the primary agent** running in the main chat. **Subagents** spawned via `runSubagent` are stateless, single-task workers. They should:
- Complete their assigned task and return results
- NOT attempt to run `py -c "task = input('')"` or any TaskSync terminal commands
- NOT attempt continuous operation or task loops
- Simply report findings/results back to the primary agent

---

*Last Updated: May 9, 2026*
