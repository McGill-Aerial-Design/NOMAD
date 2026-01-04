# NOMAD - Networked Operations for MAD

**McGill Aerial Design - AEAC 2026 Competition System**

🚁 Autonomous drone system for reconnaissance and fire extinguishing missions using:
- **Platform:** Tricopter Tiltrotor
- **Computer:** NVIDIA Jetson Orin Nano  
- **Vision:** ZED 2i Stereo Camera  
- **Flight Controller:** Cube Orange (ArduPilot)  
- **Communication:** 4G/LTE + Tailscale VPN + ELRS Transparent Serial

---

## 🎯 Competition Mission Profile

### Task 1: Locate (Outdoor GPS Mode)
- Fly over search area using GPS waypoints
- Identify target with ZED camera
- Capture snapshot with gimbal stabilization
- Generate GPS coordinates and text description
- Log mission data for judges

### Task 2: Extinguish (Indoor VIO Mode)
- Navigate building interior using ZED Visual-Inertial Odometry
- Detect fires with YOLO computer vision
- Aim gimbal-mounted extinguisher at target
- Track extinguished targets to avoid revisits
- Complete mission autonomously or with WASD nudge control

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    GROUND STATION (Domain C)                    │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │         Mission Planner + NOMAD Plugin (C#)              │  │
│  │  • Task 1/2 Controls    • WASD Nudge    • Health Monitor │  │
│  │  • Video Streams        • Telemetry     • Settings       │  │
│  └──────────────────────────────────────────────────────────┘  │
│                            ↕ MAVLink + HTTP                     │
│                    ┌────────────────────┐                       │
│                    │   Tailscale VPN    │                       │
│                    │   100.x.x.x/16     │                       │
│                    └────────────────────┘                       │
└─────────────────────────────────────────────────────────────────┘
                              ↕ 4G/LTE
┌─────────────────────────────────────────────────────────────────┐
│                     DRONE (Domain A + B)                        │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              EDGE CORE (Jetson Orin Nano)                │  │
│  │  ┌────────────────┐  ┌────────────────┐  ┌────────────┐ │  │
│  │  │ Orchestrator   │  │ Vision Process │  │ Health Mon │ │  │
│  │  │ • FastAPI      │  │ • YOLO         │  │ • Watchdog │ │  │
│  │  │ • State Mgr    │  │ • ZED Tracking │  │ • Logging  │ │  │
│  │  │ • MAVLink      │  │ • Visual Servo │  │            │ │  │
│  │  └────────────────┘  └────────────────┘  └────────────┘ │  │
│  └──────────────────────────────────────────────────────────┘  │
│                            ↕ MAVLink Router                     │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │         Cube Orange Flight Controller (ArduPilot)        │  │
│  │  • GPS/VIO Fusion    • Motor Control    • Gimbal Servo   │  │
│  └──────────────────────────────────────────────────────────┘  │
│                            ↕ PWM + Serial                       │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  Motors  │  Servos  │  ZED 2i Camera  │  LiDAR  │  ELRS  │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📁 Repository Structure

```
NOMAD/
├── docs/                   # Architecture and implementation docs
│   ├── architecture.md     # System design document
│   ├── PRD.md              # Product Requirements Document
│   ├── IMPLEMENTATION_SUMMARY.md
│   ├── TAILSCALE_SETUP.md
│   └── COMPETITION_QUICK_REFERENCE.md
│
├── config/                 # Configuration files
│   ├── landmarks.json      # Task 1 target coordinates
│   ├── env/                # Environment templates
│   │   └── jetson.env.example
│   ├── params/             # ArduPilot parameters
│   └── profiles/           # Flight mode profiles
│
├── edge_core/              # Domain B: Jetson software (Python 3.13)
│   ├── main.py             # FastAPI orchestrator entry point
│   ├── api.py              # REST API endpoints
│   ├── state.py            # State manager with thread safety
│   ├── mavlink_interface.py  # MAVLink communication
│   ├── task1.py            # Recon/snapshot logic
│   ├── task2.py            # Fire detection/exclusion logic
│   ├── vision_process.py   # Vision subprocess manager
│   ├── zed_interface.py    # ZED camera wrapper
│   ├── visual_servoing.py  # Gimbal control
│   ├── hardware_monitor.py # CPU/GPU/Temp + Watchdog
│   ├── logging_service.py  # Mission log writer
│   ├── geospatial.py       # GPS calculations
│   ├── ipc.py              # ZMQ inter-process communication
│   ├── time_manager.py     # Monotonic timestamps
│   ├── models.py           # Pydantic data models
│   ├── mocks.py            # Simulation/test mocks
│   └── requirements.txt    # Python dependencies
│
├── transport/              # Domain A: MAVLink routing
│   └── mavlink_router/
│       ├── main.conf       # MAVLink Router config
│       └── README.md
│
├── mission_planner/        # Domain C: Ground Control (C#)
│   ├── src/
│   │   ├── NOMADPlugin.cs           # Main plugin class
│   │   ├── NOMADControlPanel.cs     # UI control panel
│   │   ├── NOMADConfig.cs           # Plugin settings
│   │   ├── DualLinkSender.cs        # HTTP + MAVLink sender
│   │   ├── TelemetryInjector.cs     # HUD status messages
│   │   ├── WASDNudgeControl.cs      # Keyboard velocity control
│   │   ├── JetsonHealthTab.cs       # Real-time health monitor
│   │   ├── build_and_deploy.ps1     # Build automation
│   │   └── NOMADPlugin.csproj
│   ├── INTEGRATION_GUIDE.md
│   ├── FEATURE_INTEGRATION_COMPLETE.md
│   └── README.md
│
├── infra/                  # Deployment and infrastructure
│   ├── Dockerfile          # Jetson container (if used)
│   ├── mediamtx.yml        # RTSP video server config
│   ├── nomad.service       # systemd service for auto-start
│   ├── startup.sh          # Jetson startup script
│   └── startup.ps1         # Windows helper scripts
│
├── scripts/                # Development utilities
│   ├── run_dev.sh          # Start development server
│   └── run_dev.ps1
│
├── tests/                  # Test suite
│   ├── test_task1_geospatial.py
│   └── README.md
│
├── data/                   # Runtime data (gitignored)
│   └── mission_logs/       # Task execution logs
│
├── test_*.py               # Integration test scripts
├── start_server.py         # Quick server launcher
├── docker-compose.yml      # Optional containerized setup
├── NOMAD.sln               # Visual Studio solution
└── README.md               # This file
```

---

## 🚀 Quick Start

### Prerequisites

**Jetson (Drone):**
- NVIDIA Jetson Orin Nano (JetPack 5.1+)
- Python 3.13 or 3.11+
- ZED SDK 4.x
- MAVLink Router

**Ground Station (Laptop):**
- Windows 10/11
- Mission Planner 1.3.x
- Visual Studio 2022 (for plugin development)
- Tailscale client

### Installation

#### 1. Jetson Setup

```bash
# Clone repository
cd /home/nomad
git clone https://github.com/your-org/NOMAD.git
cd NOMAD

# Install Python dependencies
python3 -m pip install -r edge_core/requirements.txt

# Install system dependencies
sudo apt-get install mavlink-router

# Configure environment
cp config/env/jetson.env.example config/env/.env
nano config/env/.env  # Edit configuration

# Install systemd service
sudo cp infra/nomad.service /etc/systemd/system/
sudo systemctl enable nomad
sudo systemctl start nomad

# Install Tailscale
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up --authkey=<YOUR_KEY> --hostname=nomad-jetson

# Verify installation
sudo systemctl status nomad
tailscale status
curl http://127.0.0.1:8000/health
```

#### 2. Mission Planner Plugin Setup

```powershell
# Open PowerShell in mission_planner/src
cd mission_planner\src

# Build and deploy plugin
.\build_and_deploy.ps1

# Or build manually
msbuild NOMADPlugin.csproj /p:Configuration=Release /t:Rebuild
```

Plugin will be installed to:
- `%LOCALAPPDATA%\Mission Planner\plugins\NOMADPlugin.dll`
- `C:\Program Files (x86)\Mission Planner\plugins\NOMADPlugin.dll`

#### 3. Configure Tailscale on Ground Station

```bash
# Install Tailscale client from https://tailscale.com/download
# Login to same tailnet as Jetson
# Note the Jetson IP (e.g., 100.100.10.5)
```

#### 4. Launch System

**On Ground Station:**
1. Launch Mission Planner
2. Go to **NOMAD** menu → **Settings**
3. Set Jetson IP to Tailscale address (e.g., `100.100.10.5`)
4. Save settings
5. Open **NOMAD** menu → **Open Control Panel**

**On Jetson:**
- Service auto-starts on boot
- Check status: `sudo systemctl status nomad`
- View logs: `sudo journalctl -u nomad -f`

---

## 🎮 Usage

### Mission Planner Control Panel

**Task 1 - Locate:**
1. Fly drone to search area using GPS waypoints
2. Click **[CAP] Capture Snapshot**
3. View target description and GPS coordinates

**Task 2 - Extinguish:**
1. Before entering building: Click **[CLR] Reset Exclusion Map**
2. Enable WASD control for manual nudging (optional)
3. System automatically detects and tracks fires
4. Fires are registered and excluded from revisits

**WASD Indoor Control:**
- Enable checkbox: "Enable WASD Indoor Control"
- W/S: Forward/Backward
- A/D: Left/Right
- Q/E: Up/Down
- Speed: 0.5 m/s default (adjustable)
- ⚠️ **CAUTION:** Only use in Guided mode with RC transmitter ready

**Jetson Health Monitor:**
- Real-time CPU/GPU load and temperature
- Updates every 2 seconds
- Color-coded status (Green/Yellow/Red)

**Video Streams:**
- **Primary:** ZED navigation camera
- **Secondary:** Gimbal targeting camera
- Opens in VLC or FFplay

### API Endpoints

Base URL: `http://<jetson-tailscale-ip>:8000`

**Health Check:**
```bash
curl http://100.100.10.5:8000/health
```

**Task 1 - Capture:**
```bash
curl -X POST http://100.100.10.5:8000/api/task/1/capture
```

**Task 2 - Reset Map:**
```bash
curl -X POST http://100.100.10.5:8000/api/task/2/reset_map
```

**WebSocket (Real-time State):**
```javascript
ws://100.100.10.5:8000/ws
```

**Swagger UI:**
```
http://100.100.10.5:8000/docs
```

---

## 🛠️ Development

### Running Locally

**Start Jetson Server (Simulation Mode):**
```bash
export NOMAD_SIM_MODE=true
export PYTHONPATH=$(pwd)
python -m edge_core.main --sim --host 0.0.0.0 --port 8000
```

**Run Tests:**
```bash
# Quick health check
python test_local_simple.py

# Full integration test
python test_full.py
```

### Building Mission Planner Plugin

```powershell
cd mission_planner\src
.\build_and_deploy.ps1
```

### Debugging

**Jetson Logs:**
```bash
# Service logs
sudo journalctl -u nomad -f

# Application logs
tail -f data/mission_logs/*.json

# Hardware monitor
jtop
```

**Mission Planner:**
- Check NOMAD menu for status messages
- View telemetry in Messages tab
- Use MAVLink inspector (Ctrl+F) for WASD commands

---

## 📡 Network Architecture

### Tailscale VPN Mesh

```
Ground Station ←→ Tailscale Cloud ←→ Jetson Drone
100.100.x.1          (Internet)      100.100.x.5
```

**Advantages:**
- End-to-end encryption
- Works over 4G/LTE
- Automatic NAT traversal
- Persistent IP addresses
- Zero-config mesh networking

### Port Mapping

**Jetson Services:**
- `8000` - FastAPI Orchestrator (HTTP/WebSocket)
- `8554` - MediaMTX RTSP Server
- `14550` - MAVLink Orchestrator
- `14551` - MAVLink Vision Process

**Ground Station:**
- `14550` - Mission Planner MAVLink input
- `5760` - MAVLink Router TCP status

---

## 🔐 Security

- ✅ Tailscale VPN encryption for all remote access
- ✅ No exposed ports on public internet
- ✅ ELRS failsafe via transparent serial (bypasses internet)
- ✅ RC transmitter manual override always available
- ✅ Vision watchdog auto-restart on failure
- ✅ systemd service auto-restart on crash

**Credentials:**
- Never commit Tailscale auth keys
- Store secrets in `config/env/.env` (gitignored)
- Use environment variables for sensitive data

---

## 📊 Performance Targets

**Jetson Orin Nano:**
- CPU Usage: 10-50% nominal, <80% peak
- GPU Usage: 30-70% with YOLO active
- Temperature: 50-75°C nominal, <85°C peak
- RAM: <2GB used
- Power: 10-15W typical

**Latency:**
- Vision detection: <50ms per frame
- MAVLink command: <10ms local
- HTTP API: <100ms over Tailscale
- WASD control: <50ms round-trip

**Video Streaming:**
- Resolution: 720p @ 30fps
- Latency: 200-500ms over 4G/LTE
- Codec: H.264
- Protocol: RTSP

---

## 🏆 Competition Checklist

See [COMPETITION_QUICK_REFERENCE.md](docs/COMPETITION_QUICK_REFERENCE.md) for detailed day-of procedures.

**Pre-Flight:**
- [ ] Jetson service running (`sudo systemctl status nomad`)
- [ ] Tailscale connected (`tailscale status`)
- [ ] Mission Planner connected (green status)
- [ ] Health monitor shows OK
- [ ] Video streams working
- [ ] RC transmitter ready for failsafe
- [ ] WASD control disabled initially

**Task 1:**
- [ ] GPS lock acquired
- [ ] Fly to search area
- [ ] Visual target identification
- [ ] Click [CAP] Capture Snapshot
- [ ] Verify GPS coordinates and description

**Task 2:**
- [ ] Click [CLR] Reset Exclusion Map before entry
- [ ] Enable WASD if needed for nudging
- [ ] Monitor vision detections
- [ ] Verify fires are tracked
- [ ] Complete mission

**Post-Flight:**
- [ ] Download logs: `scp nomad@<ip>:/home/nomad/NOMAD/data/mission_logs ./`
- [ ] Review mission data
- [ ] Archive recordings

---

## 🤝 Contributing

This is a competition project for McGill Aerial Design at AEAC 2026.

**Development Workflow:**
1. Create feature branch from `main`
2. Implement changes with tests
3. Update documentation
4. Submit pull request with description

**Code Standards:**
- Python: PEP 8, type hints, docstrings
- C#: Microsoft conventions, XML docs
- Markdown: CommonMark spec
- No emojis in code (logs/docs OK)

---

## 📚 Documentation

- [Architecture Overview](docs/architecture.md)
- [Product Requirements](docs/PRD.md)
- [Implementation Summary](docs/IMPLEMENTATION_SUMMARY.md)
- [Tailscale Setup](docs/TAILSCALE_SETUP.md)
- [Competition Reference](docs/COMPETITION_QUICK_REFERENCE.md)
- [Mission Planner Integration](mission_planner/INTEGRATION_GUIDE.md)

---

## 📝 License

Proprietary - McGill Aerial Design Team  
For AEAC 2026 Competition Use

---

## 📞 Support

**Competition Day Contacts:**
- Team Lead: [Name] - [Phone]
- Software Lead: [Name] - [Phone]
- Tailscale Support: support@tailscale.com
- 4G/LTE Provider: [Carrier Support Number]

**System Status:**
- Jetson: `http://<tailscale-ip>:8000/health`
- Logs: `sudo journalctl -u nomad -f`
- Hardware: `jtop`

---

**Built with ❤️ by McGill Aerial Design**  
**Target Competition: AEAC 2026**
