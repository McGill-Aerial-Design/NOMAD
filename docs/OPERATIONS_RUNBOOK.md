# NOMAD Operations Runbook

Reference guide for operating the NOMAD system during development and competition.

**Jetson IP (Tailscale):** 100.85.121.98  
**Ground Station IP:** 100.76.127.17  
**Last Updated:** February 10, 2026

---

## 1. Current System Status

### Jetson Orin Nano

| Component | Version / Detail |
|-----------|-----------------|
| OS | Ubuntu 22.04 LTS (L4T R36.4) |
| JetPack | 6.2 |
| CUDA | 12.6 (drivers 540.4.0) |
| Docker | 29.2.1 with NVIDIA runtime |
| Python | venv at `/home/mad/NOMAD/venv/` |
| ROS2 | Humble (system-wide `/opt/ros/humble`) |
| ZED SDK | 4.2 for L4T 36.4 |
| Storage | 456GB NVMe SSD (~408GB free) |
| Tailscale | Connected (100.85.121.98) |

### Software Deployed

| Service | Status | Notes |
|---------|--------|-------|
| Edge Core API | Running | systemd `nomad.service`, port 8000, auto-starts on boot |
| Tailscale VPN | Connected | Jetson 100.85.121.98, GCS 100.76.127.17 |
| Docker + NVIDIA runtime | Configured | For `nomad_isaac_ros` container only |
| MediaMTX | Installed | Native binary at `/home/mad/bin/mediamtx`, systemd user service `mediamtx.service`, RTSP on port 8554, auto-starts on boot |
| Isaac ROS workspace | Initialized | `/home/mad/workspaces/isaac_ros-dev/` |

### Pending Hardware

These components activate automatically once physically connected:

- ZED2i Camera (USB3)
- Cube Orange Flight Controller (USB, `/dev/ttyACM0`)
- Camera Tilt Servo (GPIO 85)
- Water Shooter GPIO (GPIO 50)

### Typical Metrics (Idle)

- CPU Temp: 42-44C
- Memory: ~1.7GB used of 7.6GB (22%)
- Disk: 10% used
- Power: 4-5W draw / 15W budget

---

## 2. Service Architecture

All services run on the Jetson Orin Nano. The only Docker container in production is `nomad_isaac_ros`. Everything else runs on bare metal.

```
Jetson Orin Nano (100.85.121.98)
+-------------------------------------------------------+
|  Bare Metal                                           |
|                                                       |
|  nomad.service (systemd)                              |
|    Edge Core API (FastAPI, port 8000)                 |
|    Subsystems: health monitor, network monitor,       |
|    nav controller, servo controller, VIO pipeline     |
|                                                       |
|  MAVLink Router (host process)                        |
|    /dev/ttyACM0 -> UDP 14550 (GCS), 14551 (local)    |
|                                                       |
|  MediaMTX (bare metal, port 8554)                     |
|    RTSP endpoint: rtsp://100.85.121.98:8554/primary   |
|                                                       |
|  Tailscale VPN                                        |
|                                                       |
+-------------------------------------------------------+
|  Docker Container: nomad_isaac_ros                    |
|    ROS2 Humble + Isaac ROS + ZED ROS2 wrapper         |
|    Video streaming bridge (openh264enc, software)     |
|    nvblox (3D reconstruction, Task 2)                 |
+-------------------------------------------------------+

Ground Station (100.76.127.17)
+-------------------------------------------------------+
|  Mission Planner + NOMAD Plugin                       |
|  MAVLink telemetry via UDP 14550                      |
|  RTSP video via rtsp://100.85.121.98:8554/primary     |
+-------------------------------------------------------+
```

### Key Architecture Notes

- **Video encoding** uses software `openh264enc` -- there is no NVENC on the Orin Nano.
- **Video streaming** runs inside the Isaac ROS container. There is no separate video-stream container.
- **MediaMTX** runs on bare metal (not in Docker). It receives the encoded stream from the Isaac ROS container and serves it as RTSP.
- **MAVLink Router** runs as a host process, started by `scripts/run/start_nomad_full.sh`.
- **Edge Core** runs as systemd service `nomad.service` and auto-starts on boot.

### Ports

| Service | Port | Protocol |
|---------|------|----------|
| Edge Core API | 8000 | TCP/HTTP |
| MediaMTX RTSP | 8554 | TCP/RTSP |
| MAVLink Telemetry | 14550 | UDP |
| MAVLink Local | 14551 | UDP |
| SSH | 22 | TCP |

---

## 3. Quick Start

### SSH Into Jetson

```bash
ssh mad@100.85.121.98
```

### Start Everything (Recommended)

```bash
cd ~/NOMAD
git pull origin main
./scripts/run/start_nomad_full.sh
```

This script starts MAVLink Router, Edge Core (if not already running via systemd), and prepares the system.

### Manual Start (If Script Fails)

```bash
# 1. Ensure Edge Core is running
sudo systemctl start nomad
systemctl status nomad

# 2. Start MAVLink Router
mavlink-routerd -e 100.76.127.17:14550 -e 127.0.0.1:14551 /dev/ttyACM0 &

# 3. Start Isaac ROS (Task 2 only)
~/NOMAD/scripts/run/start_isaac_ros_auto.sh start
```

### Verify From Jetson

```bash
# Edge Core
systemctl status nomad
curl -s http://localhost:8000/health/detailed | python3 -m json.tool

# Docker containers
docker ps

# Tailscale
tailscale status

# Hardware detection
lsusb | grep -i "stereolabs\|cube"
```

### Verify From Windows GCS

```powershell
# Basic health
Invoke-WebRequest -Uri 'http://100.85.121.98:8000/health' -UseBasicParsing

# Detailed metrics
(Invoke-WebRequest -Uri 'http://100.85.121.98:8000/health/detailed' -UseBasicParsing).Content |
  ConvertFrom-Json | ConvertTo-Json -Depth 5

# Network status
(Invoke-WebRequest -Uri 'http://100.85.121.98:8000/network/status' -UseBasicParsing).Content |
  ConvertFrom-Json | ConvertTo-Json -Depth 5
```

### Ground Station Setup

1. Install Mission Planner from https://ardupilot.org/planner/docs/mission-planner-installation.html
2. Install NOMAD Plugin:
   ```powershell
   Copy-Item "NOMAD\mission_planner\src\bin\Release\NOMADPlugin.dll" `
     "C:\Program Files (x86)\Mission Planner\plugins\"
   ```
3. Install LibVLC for video:
   ```powershell
   cd NOMAD\mission_planner\packaging
   .\fetch-libvlc.ps1
   .\copy-libvlc.ps1
   ```
4. Connect: Open Mission Planner, select UDP port 14550, click CONNECT.
5. Open NOMAD panel: **View** -> **NOMAD Control Panel**.

---

## 4. Competition Day Checklist

### Phase 1: Power Up and Connect

1. Power on Jetson (5V 4A from drone PDB or 45W+ USB-C PD power bank).
2. Wait 30-60 seconds for boot and `nomad.service` to start.
3. SSH in: `ssh mad@100.85.121.98`
4. Verify Tailscale: `tailscale status`
5. Pull latest code if needed: `cd ~/NOMAD && git pull origin main`
6. Run full startup: `./scripts/run/start_nomad_full.sh`

### Phase 2: Verify Systems

7. Verify Edge Core: `curl -s http://localhost:8000/health/detailed | python3 -m json.tool`
8. Verify ZED camera: `lsusb | grep Stereolabs`
9. Verify flight controller: `ls /dev/ttyACM0`
10. From GCS, confirm MAVLink connection in Mission Planner (shows "Connected").
11. From GCS, verify API: `Invoke-WebRequest -Uri 'http://100.85.121.98:8000/health' -UseBasicParsing`

### Phase 3: Task 1 (Outdoor Recon)

12. In NOMAD panel, select **Task 1: Outdoor Recon**.
13. Verify GPS: 3D fix, HDOP < 1.5, satellites > 10.
14. Load and upload waypoint mission via Mission Planner.
15. At each target, use the Capture button or call:
    ```bash
    curl -X POST http://100.85.121.98:8000/api/task/1/capture
    ```

### Phase 4: Task 2 (Indoor Extinguish)

16. Start Isaac ROS:
    ```bash
    curl -X POST http://100.85.121.98:8000/api/isaac/start
    ```
    Or via SSH: `~/NOMAD/scripts/run/start_isaac_ros_auto.sh start`

17. Verify VIO:
    ```bash
    curl -s http://100.85.121.98:8000/api/vio/status | python3 -m json.tool
    ```

18. Reset VIO origin at takeoff position:
    ```bash
    curl -X POST http://100.85.121.98:8000/api/vio/reset_origin
    ```

19. Verify video stream is available at `rtsp://100.85.121.98:8554/primary`.

20. During flight, monitor VIO confidence (keep above 0.8).

21. Register target hits:
    ```bash
    curl -X POST http://100.85.121.98:8000/api/task/2/target_hit \
      -H "Content-Type: application/json" \
      -d '{"x": 1.5, "y": 2.3, "z": 0.5}'
    ```

### Emergency Procedures

- **Loss of VIO during flight:** Immediately switch to STABILIZE mode via RC, hover, navigate visually to exit.
- **Loss of communication:** MAVLink failsafe triggers RTL after 30s. Indoors: switch to STABILIZE and land manually.
- **Thermal throttling:** Land if VIO confidence drops below 0.5. Allow 5 minutes cooldown.
- **Emergency shutdown:** `ssh mad@100.85.121.98 "sudo shutdown now"` -- or pull power if unreachable.

---

## 5. Hardware Connections

### Physical Setup

| Component | Connection | Jetson Port | Device Path |
|-----------|-----------|-------------|-------------|
| ZED 2i Camera | USB-C to USB-A (USB 3.0 cable) | Top USB 3.0 port (blue) | Detected via `lsusb` |
| Cube Orange FC | USB-C to USB-A | Bottom USB port | `/dev/ttyACM0` |
| Camera Tilt Servo | GPIO PWM | gpiochip0/line85 | Controlled via Edge Core API |
| Water Shooter | GPIO output | gpiochip0/line50 | Controlled via Edge Core API |
| Jetson Power | 5V 4A from PDB or USB-C PD 45W+ | Power input | -- |

### Mounting Notes

- Jetson: near center of gravity, USB ports accessible, vibration-dampened (rubber grommets).
- ZED 2i: forward-facing, level with drone frame.

### Verification Commands

```bash
# ZED camera
lsusb | grep -i stereolabs
/usr/local/zed/tools/ZED_Explorer   # Visual test

# Flight controller
ls /dev/ttyACM0

# Servo and GPIO
curl -s http://localhost:8000/api/servo/status | python3 -m json.tool

# Tilt camera to 45 degrees
curl -X POST 'http://localhost:8000/api/servo/camera/tilt?angle=45'

# Trigger water shooter for 500ms
curl -X POST 'http://localhost:8000/api/servo/shooter/trigger?duration_ms=500'
```

### Network Verification

```bash
# Tailscale connected
tailscale status

# GCS reachable
ping 100.76.127.17

# MAVLink flowing (check Mission Planner for heartbeat)
curl -s http://localhost:8000/status | python3 -m json.tool
```

---

## 6. Troubleshooting

### Edge Core Not Responding

```bash
systemctl status nomad
journalctl -u nomad --no-pager -n 50

# Restart service
sudo systemctl restart nomad

# Manual test (if systemd fails)
cd /home/mad/NOMAD
/home/mad/NOMAD/venv/bin/python -m edge_core.main
```

### MAVLink Not Connecting

```bash
# Check USB device
ls -la /dev/ttyACM*

# If missing, check permissions
sudo usermod -a -G dialout $USER
# Requires logout/login

# Restart MAVLink Router
pkill mavlink-routerd
mavlink-routerd -e 100.76.127.17:14550 -e 127.0.0.1:14551 /dev/ttyACM0 &
```

### ZED Camera Not Detected

```bash
lsusb | grep -i stereolabs
dmesg | tail -20   # Check for USB enumeration errors
```

If not found: verify cable is USB 3.0, try a different port, reboot Jetson.

### Isaac ROS Container Won't Start

```bash
docker ps -a | grep nomad
docker logs nomad_isaac_ros --tail 50

# Remove and restart
docker rm -f nomad_isaac_ros
~/NOMAD/scripts/run/start_isaac_ros_auto.sh start
```

### VIO Tracking Lost

1. Hold position and hover.
2. Ensure adequate lighting.
3. Check for camera obstructions.
4. Reset origin if needed: `curl -X POST http://localhost:8000/api/vio/reset_origin`

### Tailscale Disconnected

```bash
sudo tailscale up
tailscale status
```

### High Temperature (> 75C)

```bash
curl -s http://localhost:8000/health/detailed | python3 -m json.tool
```

Reduce processing load, improve airflow, or add heatsink. Land immediately if VIO confidence drops below 0.5.

### ROS2 Not Found

```bash
source /opt/ros/humble/setup.bash
# Or logout/login to load /etc/profile.d/nomad-env.sh
```

### Cube Orange Permissions

```bash
ls /dev/ttyACM*
sudo usermod -a -G dialout $USER
# Logout and login required after this
```

---

## 7. Configuration Files Reference

| File | Location on Jetson | Purpose |
|------|--------------------|---------|
| Edge Core systemd service | `/etc/systemd/system/nomad.service` | Service definition, auto-start |
| Environment (system-wide) | `/etc/profile.d/nomad-env.sh` | CUDA and ROS2 paths |
| Edge Core env vars | `/home/mad/NOMAD/config/env/jetson.env` | IPs, ports, feature flags |
| Local env overrides | `/home/mad/NOMAD/.env` | Local overrides |
| Docker Compose | `/home/mad/NOMAD/docker-compose.yml` | Container orchestration |
| MediaMTX config | `/home/mad/NOMAD/infra/mediamtx.yml` | RTSP server settings |
| Task 1 params | `/home/mad/NOMAD/config/params/task1_gps.param` | GPS task configuration |
| Task 2 params | `/home/mad/NOMAD/config/params/task2_vio.param` | VIO task configuration |
| Task 1 profile | `/home/mad/NOMAD/config/profiles/task1_outdoor.params` | Outdoor flight profile |
| Task 2 profile | `/home/mad/NOMAD/config/profiles/task2_indoor.params` | Indoor flight profile |
| Landmarks | `/home/mad/NOMAD/config/landmarks.json` | Known landmark positions |

### Key API Endpoints

> Full Swagger docs: `http://100.85.121.98:8000/docs`

#### System / Services
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Service info |
| `/health` | GET | System health overview |
| `/health/detailed` | GET | Full Jetson metrics (temp, memory, disk, CPU, GPU) |
| `/status` | GET | Complete system state |
| `/api/services/status` | GET | systemd / process status for nomad, mediamtx, mavlink-routerd |

#### Task 1 (Outdoor Recon)
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/task/1/target/capture` | POST | Trigger target detection + description |
| `/api/task/1/target/save` | POST | Save targets to `Task_1_MAD_targets.txt` |
| `/api/task/1/target/clear` | POST | Clear all captured targets |
| `/api/task/1/target/list` | GET | List captured targets |
| `/api/task/1/target/model` | GET | Building model summary |
| `/api/task/1/capture` | POST | Legacy capture fallback |
| `/api/task/1/building/corner` | POST | Save one building corner GPS |
| `/api/task/1/building/corners` | GET | List saved building corners |
| `/api/task/1/building/corners/apply` | POST | Rebuild building model from corners |

#### Task 2 (Indoor VIO)
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/task/2/reset_map` | POST | Clear exclusion map |
| `/api/task/2/target_hit` | POST | Register target position |
| `/api/task/2/exclusion_map` | GET | Get hit targets |

#### VIO
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/vio/status` | GET | VIO pipeline health |
| `/api/vio/pose` | GET | Current position/orientation |
| `/api/vio/trajectory` | GET | Path history |
| `/api/vio/reset_origin` | POST | Reset VIO tracking origin |
| `/api/vio/calibration` | GET | ZED calibration state |

#### Navigation / Isaac ROS
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/nav/status` | GET | Navigation controller status |
| `/api/nav/velocity` | POST | Send velocity command |
| `/api/nav/stop` | POST | Emergency stop |
| `/api/isaac/status` | GET | Isaac ROS container status |
| `/api/isaac/start` | POST | Start Isaac ROS container |
| `/api/isaac/stop` | POST | Stop Isaac ROS container |

#### Servo / Spray / Video / Terminal / Network
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/servo/status` | GET | Servo and GPIO status |
| `/api/servo/camera/tilt?angle=N` | POST | Set camera servo angle (0-180) |
| `/api/servo/shooter/trigger?duration_ms=N` | POST | Trigger water shooter GPIO |
| `/api/spray/status` | GET | Spray controller state |
| `/api/spray/trigger` | POST | Trigger spray sequence |
| `/api/spray/abort` | POST | Abort spray sequence |
| `/api/video/status` | GET | Video pipeline status |
| `/api/video/start` | POST | Start video streaming |

---

*NOMAD -- MAD Team AEAC 2026*
