# NOMAD - Networked Operations for MAD

**McGill Aerial Design - AEAC 2026 Competition System**

🚁 Drone system for two distinct competition tasks:

| Task | Configuration | Computer | Navigation |
|------|--------------|----------|------------|
| **Task 1** (Outdoor Recon) | ZED 2i camera | Orin Nano (imaging only) | GPS/RTK (pilot-only) |
| **Task 2** (Indoor Extinguish) | With Jetson | Orin Nano | ZED VIO |

---

## 🎯 Task Overview

### Task 1: Outdoor Reconnaissance
- **Pilot-only operation** - no autonomous navigation
- Jetson Orin Nano + ZED 2i camera mounted for target imagery
- Images used to generate text descriptions (out of scope for this repo)
- GPS/RTK positioning via ELRS telemetry
- RTCM corrections through Mission Planner

### Task 2: Indoor Fire Extinguishing  
- **Jetson-powered autonomous** operation
- ZED 2i Visual-Inertial Odometry
- YOLO target detection
- 4G/LTE + Tailscale communication

---

## 🏗️ System Architecture

### Task 1 (Jetson camera only)
```
┌─────────────────────────────────────────────────────────────────┐
│                    GROUND STATION                               │
│  Mission Planner ←──ELRS Gemini──→ Cube Orange ←──GPS──→ RTK   │
└─────────────────────────────────────────────────────────────────┘
```
Jetson Orin Nano + ZED 2i camera are mounted for imaging only (no autonomous navigation).

### Task 2 (With Jetson)
```
┌─────────────────────────────────────────────────────────────────┐
│                    GROUND STATION                               │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │         Mission Planner + NOMAD Plugin (C#)              │  │
│  │  • Jetson Health     • WASD Nudge    • Task 2 Controls   │  │
│  └──────────────────────────────────────────────────────────┘  │
│                            ↕ MAVLink + HTTP                     │
│                    ┌────────────────────┐                       │
│                    │   Tailscale VPN    │                       │
│                    │   (4G/LTE)         │                       │
│                    └────────────────────┘                       │
└─────────────────────────────────────────────────────────────────┘
                              ↕
┌─────────────────────────────────────────────────────────────────┐
│                     DRONE                                       │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              EDGE CORE (Jetson Orin Nano)                │  │
│  │  • FastAPI Server     • ZED VIO      • YOLO Detection    │  │
│  │  • State Manager      • Gimbal PID   • Exclusion Map     │  │
│  └──────────────────────────────────────────────────────────┘  │
│                            ↕ MAVLink Router                     │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │         Cube Orange Flight Controller (ArduPilot)        │  │
│  │  • EKF with VIO fusion   • ELRS backup receiver          │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📁 Repository Structure

```
NOMAD/
├── docs/                   # Documentation
│   ├── architecture.md     # System design
│   └── PRD.md              # Product requirements
│
├── edge_core/              # Jetson software (Task 2 autonomy + Task 1 imaging support)
│   ├── main.py             # Entry point
│   ├── api.py              # REST API endpoints
│   ├── state.py            # State manager
│   ├── mavlink_interface.py  # Flight controller comms
│   ├── time_manager.py     # Time synchronization
│   ├── geospatial.py       # GPS calculations
│   └── models.py           # Data models
│
├── tailscale/              # VPN configuration (Task 2)
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
│       ├── NOMADPlugin.cs         # Plugin entry point
│       ├── NOMADFullPage.cs       # Full-page control interface
│       ├── NOMADControlPanel.cs   # Quick access panel
│       ├── EmbeddedVideoPlayer.cs # Built-in video streaming
│       ├── JetsonTerminalControl.cs # Remote terminal
│       └── ...
│
├── config/                 # Configuration files
│   ├── params/             # ArduPilot parameter files
│   └── env/                # Environment configs
│
├── infra/                  # Deployment configs
│   ├── Dockerfile
│   └── nomad.service
│
└── scripts/                # Dev scripts
    └── run_dev.sh
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
| Tailscale VPN | N/A | ✅ Ready |
| ZED 2i Camera | ✅ Ready | ✅ Ready |
| YOLO Detection | N/A | ⏳ In Progress |
| Mission Planner Plugin | ✅ Ready | ✅ Ready |
| Embedded Video | ✅ Ready | ✅ Ready |
| Remote Terminal | ✅ Ready | ✅ Ready |
| Health Monitoring | ✅ Ready | ✅ Ready |
