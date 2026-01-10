# NOMAD - Networked Operations for MAD

**McGill Aerial Design - AEAC 2026 Competition System**

🚁 Drone system for two distinct competition tasks:

| Task | Configuration | Computer | Navigation |
|------|--------------|----------|------------|
| **Task 1** (Outdoor Recon) | No Jetson | None | GPS/RTK |
| **Task 2** (Indoor Extinguish) | With Jetson | Orin Nano | ZED VIO |

---

## 🎯 Task Overview

### Task 1: Outdoor Reconnaissance
- **Pilot-only operation** - no edge compute
- GPS/RTK positioning via ELRS telemetry
- RTCM corrections through Mission Planner
- **Jetson is NOT mounted on drone**

### Task 2: Indoor Fire Extinguishing  
- **Jetson-powered autonomous** operation
- ZED 2i Visual-Inertial Odometry
- YOLO target detection
- 4G/LTE + Tailscale communication

---

## 🏗️ System Architecture

### Task 1 (No Jetson)
```
┌─────────────────────────────────────────────────────────────────┐
│                    GROUND STATION                               │
│  Mission Planner ←──ELRS Gemini──→ Cube Orange ←──GPS──→ RTK   │
└─────────────────────────────────────────────────────────────────┘
```

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
├── edge_core/              # Jetson software (Task 2 only)
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
│       ├── NOMADPlugin.cs
│       └── ...
│
├── config/                 # Configuration files
│   └── params/             # ArduPilot parameter files
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

### Task 1 Setup (No Jetson)
```bash
# Ground station only
1. Connect ELRS Gemini TX to computer
2. Open Mission Planner
3. Connect to drone via ELRS
4. Configure RTK/NTRIP for corrections
5. Fly with GPS waypoints
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
3. Verify Jetson health in plugin tab
```

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
| Edge Core API | N/A | ✅ Ready |
| Tailscale VPN | N/A | ✅ Ready |
| ZED VIO | N/A | ⏳ In Progress |
| YOLO Detection | N/A | ⏳ In Progress |
| Mission Planner Plugin | ✅ Basic | ⏳ In Progress |
