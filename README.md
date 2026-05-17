# NOMAD - Networked Operations for MAD

**McGill Aerial Design - AEAC 2026 Competition System**

Drone system for two distinct competition tasks:

| Task | Environment | Flight Control | Positioning | Jetson Role |
|------|-------------|----------------|-------------|-------------|
| **Task 1** (Outdoor Recon) | Outdoor | RC Pilot (ELRS) | GPS + baro (optical flow backup) | Video + target localization |
| **Task 2** (Fire Extinguishing) | Outdoor + Indoor | RC Pilot (manual indoors); autonomous spray for 1 outdoor target | GPS + baro (optical flow backup) | Video + autonomous aim/spray on 1 target |

---

## Task Overview

### Task 1: Outdoor Reconnaissance
- **Traditional RC pilot control** via ELRS directly to ArduPilot
- Jetson provides video streaming and target imaging only
- GPS/RTK positioning for outdoor operation
- No autonomous navigation

### Task 2: Fire Extinguishing (outdoor + indoor)
- **GPS-based outdoor approach + manual indoor piloting** — no VIO.
- ArduPilot in GUIDED for the outdoor autonomous segment; STABILIZE / ALT_HOLD indoors.
- ZED 2i provides RGB + depth for target detection and autonomous aim.
- Autonomous spray sequence runs on **one (outdoor) target** to satisfy the
  20-pt autonomy gate; remaining targets are sprayed manually.
- Pilot retains RC control at all times. Kill switch = CH5 arming (disarm cuts motors). RC12 = pilot-triggered auto-LAND for soft recoveries.

---

## System Architecture

### Task 1: Outdoor (RC Pilot Control)
```
+---------------------------------------------------------------+
|                    GROUND STATION                              |
|  Mission Planner (telemetry display)                          |
|  ELRS TX (RC control)                                          |
+---------------------------------------------------------------+
                     |
              ELRS Radio Link
                     |
+---------------------------------------------------------------+
|                     DRONE                                      |
|  +-- Cube Orange (ArduPilot) <-- ELRS RX                      |
|  |   Standard flight modes (pilot control)                    |
|  |                                                             |
|  +-- Jetson Orin Nano (video only)                            |
|      +-- ZED camera streaming                                 |
|      +-- Tailscale (video/status to ground)                   |
+---------------------------------------------------------------+
```

### Task 2: Outdoor approach → manual indoor
```
+---------------------------------------------------------------+
|                    GROUND STATION                              |
|  Mission Planner + NOMAD Plugin                                |
|  +-- RTM checklist + spray status                             |
|  +-- Manual flight via ELRS                                   |
|            Tailscale VPN (4G/LTE for video + API)             |
+---------------------------------------------------------------+
                     |
+---------------------------------------------------------------+
|                    JETSON ORIN NANO                            |
|  +-- Edge Core (Python)                                        |
|  |   +-- SprayController (1 autonomous target)                 |
|  |   +-- ServoController (camera tilt + water pump)            |
|  |   +-- MavlinkService (GUIDED for 1-target auto)             |
|  |                                                             |
|  +-- Isaac ROS container (ZED wrapper + video bridge)          |
+---------------------------------------------------------------+
                     |
+---------------------------------------------------------------+
|               CUBE ORANGE (ArduPilot)                          |
|  RC pilot in command. Modes: STABILIZE / ALT_HOLD / LOITER /   |
|  GUIDED (transient, during the one autonomous spray).          |
|  Kill = CH5 disarm; RC12 = auto-LAND (CONOPS §4.9.c).          |
+---------------------------------------------------------------+
```

### Task 1 vs Task 2 Differences
| Component | Task 1 (Outdoor) | Task 2 (Out → In) |
|-----------|------------------|-------------------|
| Flight Control | RC Pilot via ELRS | RC Pilot via ELRS + GUIDED for 1 auto spray |
| ArduPilot Mode | LOITER / AUTO / GUIDED | STABILIZE / ALT_HOLD indoors; GUIDED outdoors |
| Position Source | GPS + baro (OF backup) | GPS + baro (OF backup) — no VIO |
| Isaac ROS | ZED wrapper for capture | ZED wrapper for spray aim |
| Jetson Role | Video + target localization | Video + 1-target autonomous aim/spray |
| Kill Switch | CH5 disarm (PWM cut) | CH5 disarm (PWM cut) |
| Auto-LAND   | RC12 → LAND mode | RC12 → LAND mode |

---

## Repository Structure

```
NOMAD/
├── docs/                   # Documentation
│   ├── architecture.md     # System design
│   ├── OPERATIONS_RUNBOOK.md # Operations guide
│   ├── JETSON_DEPLOYMENT.md  # Deployment guide
│   └── PRD.md              # Product requirements
│
├── edge_core/              # Jetson software (Task 2 autonomy + Task 1 imaging support)
│   ├── main.py             # Entry point
│   ├── api.py              # REST API endpoints (~8K lines, all tags)
│   ├── state.py            # State manager
│   ├── mavlink_interface.py  # MAVLink flight controller comms
│   ├── nav_controller.py     # Velocity/position command logic
│   ├── servo_controller.py # Cube Orange servo / relay commands
│   ├── spray_controller.py # Fire-extinguisher spray control
│   ├── operational_mode.py # Operational mode state machine
│   ├── video_stream_manager.py # Video bridge / overlay management
│   ├── isaac_ros_bridge.py # Isaac ROS / nvblox lifecycle
│   ├── health_monitor.py   # Jetson hardware monitoring
│   ├── time_manager.py     # Time synchronization
│   ├── geospatial.py       # GPS/ENU conversions
│   ├── models.py           # Pydantic data models
│   ├── gdrive_upload.py    # Google Drive upload helpers
│   ├── ipc.py              # ZMQ/IPC for high-rate ROS data
│   ├── logging_service.py  # Structured logging utilities
│   ├── ros_http_bridge.py  # ROS <-> HTTP bridge
│   ├── target_localizer/   # HSV circle detection + building model
│   └── ros/                # ROS2 utility nodes (VIO, video, obstacles)
│
├── tailscale/              # VPN configuration
│   ├── SETUP.md            # Installation guide
│   ├── src/                # Python managers
│   ├── scripts/            # Setup/watchdog scripts
│   └── config/             # Systemd services
│
├── transport/              # MAVLink routing
│   └── mavlink_router/
│       └── main.conf       # Router config
│
├── mission_planner/        # Ground Control Plugin (C#)
│   └── src/
│       ├── NOMADPlugin.cs            # Plugin entry point
│       ├── NOMADMainScreen.cs        # Main plugin screen
│       ├── NOMADDashboardView.cs     # Main dashboard view
│       ├── NOMADTask1View.cs       # Task 1 capture / submit UI
│       ├── NOMADTask2View.cs       # Task 2 VIO / WASD UI
│       ├── ServiceControlPanel.cs  # Service status panel
│       ├── EnhancedHealthDashboard.cs # Health + network display
│       ├── JetsonConnectionManager.cs # API client
│       ├── JetsonTerminalControl.cs # Remote terminal
│       ├── EmbeddedVideoPlayer.cs   # Built-in video streaming
│       ├── LinkHealthPanel.cs       # MAVLink dual-link health
│       ├── SLAM3DView.cs + SLAM3D/ # 3D nvblox mesh viewer
│       ├── Task1UploadPanel.cs      # Target grid + Google Drive upload
│       ├── PayloadControlPanel.cs   # Payload drop / water shooter
│       └── ...
│
├── config/                 # Configuration files
│   ├── params/             # ArduPilot parameter files
│   └── env/                # Environment configs
│
├── infra/                  # Deployment configs
│   ├── Dockerfile
│   ├── mediamtx.yml        # MediaMTX RTSP server config
│   └── nomad.service       # systemd service
│
└── scripts/                # Dev / setup / runtime scripts
    ├── build/              # Build scripts
    ├── run/                # Runtime/startup scripts
    ├── setup/              # One-time setup scripts
    └── dev/                # Development tools
```

---

## 🚀 Quick Start

### Task 1 Setup (Jetson camera)
```bash
# Ground station + Jetson camera (imaging only)
1. Connect ELRS Gemini TX to computer
2. Open Mission Planner
3. Connect to drone via ELRS
4. Configure RTK/NTRIP for corrections
5. Use NOMAD → Open Full Control Page for capture controls
6. Fly with GPS waypoints
```

### Task 2 Setup (With Jetson)
```bash
# On Jetson
cd NOMAD
pip install -r edge_core/requirements.txt
sudo tailscale/scripts/setup.sh --authkey <KEY>
python -m edge_core.main --host 0.0.0.0 --port 8000

# On Ground Station
1. Connect via Tailscale IP
2. Open Mission Planner with NOMAD plugin
3. Use NOMAD menu → Open Full Control Page
4. Check Jetson health in Health tab
5. Use embedded video or terminal as needed
```

---

## 🎮 Mission Planner Plugin Features

### Full Control Page
- **Dashboard**: System overview, quick actions, connection status
- **Task 1 Tab**: GPS capture controls, waypoint management
- **Task 2 Tab**: VIO controls, WASD nudge, exclusion map
- **Video Tab**: Embedded RTSP streaming (no VLC needed)
- **Terminal Tab**: Remote command execution on Jetson
- **Health Tab**: CPU/GPU temps, memory, disk, network status

### Quick Access
- Right-click FlightData map → NOMAD Full Control
- Menu bar → NOMAD → Open Full Control Page
- Keyboard shortcut support (configurable)

---

## 📡 Communication Links

| Link | Task 1 | Task 2 |
|------|--------|--------|
| **ELRS 2.4GHz** | Primary control | Backup control |
| **ELRS 900MHz** | Extended range | Backup control |
| **4G/LTE** | Not used | Primary data |
| **Tailscale** | Not used | API + Video |

---

## 📋 Status

| Component | Task 1 | Task 2 |
|-----------|--------|--------|
| ArduPilot Integration | ✅ Ready | ✅ Ready |
| ELRS Telemetry | ✅ Ready | ✅ Ready |
| Edge Core API | ✅ Ready | ✅ Ready |
| Tailscale VPN |  ✅ Ready | ✅ Ready |
| ZED 2i Camera | ✅ Ready | ✅ Ready |
| YOLO Detection | ⏳ In Progress | ⏳ In Progress |
| Mission Planner Plugin | ✅ Ready | ✅ Ready |
| Embedded Video | ✅ Ready | ✅ Ready |
| Remote Terminal | ✅ Ready | ✅ Ready |
| Health Monitoring | ✅ Ready | ✅ Ready |
