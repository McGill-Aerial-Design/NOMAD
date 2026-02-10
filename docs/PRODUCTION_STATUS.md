# NOMAD Jetson - Production System Status

**Last Updated:** February 9, 2026  
**Jetson IP:** 100.85.121.98 (Tailscale)  
**Status:** PRODUCTION READY (awaiting hardware)

---

## ✅ Fully Configured Systems

### Core Services
- **Edge Core API** - Systemd service (`nomad.service`), auto-starts on boot
  - Port: 8000
  - Health endpoint: `http://localhost:8000/health`
  - All subsystems initialized: health monitor, network monitor, nav controller, servo controller
  - Status: RUNNING

- **Tailscale VPN** - Connected to network
  - Jetson: 100.85.121.98
  - GCS: 100.76.127.17 (reachable)
  - Status: CONNECTED

- **Docker** - v29.2.1 with NVIDIA runtime
  - Isaac ROS container (nomad_isaac_ros) for ZED + ROS2
  - Status: RUNNING

### Development Environment
- **Python** - venv at `/home/mad/NOMAD/venv/`
  - All dependencies installed (pyserial, piexif, pymavlink, etc.)
  - Status: READY

- **ROS2 Humble** - Native installation
  - System-wide at `/opt/ros/humble`
  - Packages: zed-msgs, robot-localization, tf2, colcon
  - Status: INSTALLED

- **CUDA 12.6** - Toolkit installed
  - nvcc available at `/usr/local/cuda/bin/nvcc`
  - Environment configured via `/etc/profile.d/nomad-env.sh`
  - Status: CONFIGURED

- **ZED SDK 4.2** - For L4T 36.4
  - Libraries: `/usr/local/zed/lib/`
  - Tools: `/usr/local/zed/tools/` (ZED_Explorer, Calibration, etc.)
  - Status: INSTALLED

### Workspace
- **Isaac ROS Workspace** - `/home/mad/workspaces/isaac_ros-dev/`
  - isaac_ros_common cloned in src/
  - Status: INITIALIZED (awaiting Docker approach for full build)

---

## ⚠️ Pending Hardware

### Not Yet Connected
- ❌ ZED2i Camera (USB3)
- ❌ Cube Orange Flight Controller (USB /dev/ttyACM0)  
- ❌ Camera Tilt Servo (GPIO 85)
- ❌ Water Shooter GPIO (GPIO 50)

**When hardware is connected**, all subsystems will activate automatically.

---

## 🚀 Quick Start Commands

### Check System Status
```bash
# Full system check
./scripts/startup.sh

# Edge Core health
curl http://localhost:8000/health/detailed | jq

# Network status
curl http://localhost:8000/network/status | jq
```

### Service Management
```bash
# Edge Core
systemctl status nomad
journalctl -u nomad -f

# Docker
docker ps
docker logs nomad-mediamtx

# Tailscale
tailscale status
```

### Development
```bash
# Activate Python venv
source ~/NOMAD/venv/bin/activate

# Source ROS2
source /opt/ros/humble/setup.bash

# Build Isaac ROS workspace (when ready)
cd ~/workspaces/isaac_ros-dev
colcon build
```

---

## 📝 Configuration Files

| File | Purpose |
|------|---------|
| `/etc/systemd/system/nomad.service` | Edge Core systemd service |
| `/etc/profile.d/nomad-env.sh` | System-wide CUDA/ROS2 environment |
| `/home/mad/NOMAD/config/env/jetson.env` | Edge Core environment variables |
| `/home/mad/NOMAD/.env` | Local environment overrides |
| `/home/mad/NOMAD/docker-compose.yml` | Container orchestration |

---

## 🔧 Troubleshooting

### Edge Core Not Responding
```bash
systemctl restart nomad
journalctl -u nomad --no-pager -n 50
```

### Tail scale Disconnected
```bash
sudo tailscale up
tailscale status
```

### Docker Container Issues
```bash
docker ps -a
docker compose up -d isaac-ros
docker logs nomad_isaac_ros
```

### ROS2 Not Found
```bash
source /opt/ros/humble/setup.bash
# Or logout/login to load /etc/profile.d/nomad-env.sh
```

---

## 🎯 Next Steps (When Hardware Arrives)

1. **Connect ZED2i Camera**
   - Verify detection: `lsusb | grep Stereolabs`
   - Test: `/usr/local/zed/tools/ZED_Explorer`
   - Check Edge Core VIO status: `curl localhost:8000/api/vio/status`

2. **Connect Flight Controller**
   - Check device: `ls /dev/ttyACM0`
   - Verify MAVLink: `curl localhost:8000/status`
   - Test telemetry from Mission Planner

3. **Test Servo Control**
   - Tilt camera: `curl -X POST localhost:8000/api/servo/camera/tilt?angle=45`
   - Check status: `curl localhost:8000/api/servo/status`

4. **Launch Video Streaming**
   - Video bridge runs inside Isaac ROS container (managed by Edge Core)
   - Start via API: `curl -X POST localhost:8000/api/video/start`
   - Connect Mission Planner to `rtsp://100.85.121.98:8554/primary`

5. **Isaac ROS (Optional)**
   - Follow guide: `docs/ISAAC_ROS_ZED_SETUP.md`
   - Alternative: Use Docker approach when base image is fixed

---

## 📊 System Metrics

Run from Windows GCS:
```powershell
(Invoke-WebRequest -Uri 'http://100.85.121.98:8000/health/detailed' -UseBasicParsing).Content | ConvertFrom-Json | ConvertTo-Json
```

Current typical metrics:
- CPU Temp: 42-44°C
- GPU: Available  
- Memory: 7.6GB total, ~1.7GB used (22%)
- Disk: 456GB total, 408GB free (10% used)
- Power: 4-5W draw / 15W budget

---

## ✅ Production Readiness Checklist

- [x] Edge Core API running
- [x] Tailscale connected
- [x] Docker configured
- [x] Python environment setup
- [x] ROS2 Humble installed
- [x] CUDA environment configured  
- [x] ZED SDK installed
- [x] Startup script created
- [x] System documentation complete
- [x] SSH passwordless from Windows
- [x] Systemd service auto-starts
- [ ] ZED camera connected (pending hardware)
- [ ] Flight controller connected (pending hardware)
- [ ] Full end-to-end test (pending hardware)

---

**System is production-ready and waiting for hardware delivery.**

For detailed setup guides, see:
- [docs/HARDWARE_CHECKLIST.md](docs/HARDWARE_CHECKLIST.md) - Pre-flight verification
- [docs/ISAAC_ROS_ZED_SETUP.md](docs/ISAAC_ROS_ZED_SETUP.md) - Isaac ROS + ZED guide
- [AGENTS.md](AGENTS.md) - Full system reference for AI agents
