# NOMAD Competition Day Setup Guide

This guide covers everything needed to set up NOMAD right before the AEAC 2026 competition.

---

## Table of Contents

1. [Pre-Competition Checklist](#pre-competition-checklist)
2. [Hardware Setup](#hardware-setup)
3. [Network Configuration](#network-configuration)
4. [Jetson Orin Nano Setup](#jetson-orin-nano-setup)
5. [Ground Station Setup](#ground-station-setup)
6. [System Verification](#system-verification)
7. [Task 1 (Outdoor Recon) Setup](#task-1-outdoor-recon-setup)
8. [Task 2 (Indoor Extinguish) Setup](#task-2-indoor-extinguish-setup)
9. [Troubleshooting](#troubleshooting)
10. [Emergency Procedures](#emergency-procedures)

---

## Pre-Competition Checklist

### Hardware

- [ ] Jetson Orin Nano with power supply (5V 4A or battery)
- [ ] ZED 2i stereo camera with USB-C cable
- [ ] CubePilot Orange+ flight controller
- [ ] USB-C to USB-A cable (Jetson to CubePilot)
- [ ] Ethernet cable (optional - for initial setup)
- [ ] Drone frame with motors, ESCs, propellers
- [ ] LiPo battery (fully charged)
- [ ] Spray mechanism (Task 2)
- [ ] SiK Telemetry Radio pair (433MHz or 915MHz)
- [ ] RC transmitter (bound to receiver)

### Software

- [ ] Mission Planner with NOMAD plugin installed
- [ ] Tailscale account and auth key ready
- [ ] Competition venue WiFi credentials (if available)

---

## Hardware Setup

### 1. Mount Jetson Orin Nano

```
Position: Near center of gravity
Orientation: USB ports accessible
Mounting: Vibration-dampened (rubber grommets recommended)
```

### 2. Connect ZED 2i Camera

```
Cable: USB 3.0 (USB-C to USB-A)
Port: Top USB 3.0 port on Jetson (blue port)
Position: Forward-facing, level with drone frame
```

### 3. Connect CubePilot Flight Controller

```
Cable: USB-C to USB-A
Port: Bottom USB port on Jetson
Device: Will appear as /dev/ttyACM0
```

### 4. Power Connections

```
Jetson: 5V 4A from drone power distribution board
        OR USB-C PD power bank (45W minimum)
CubePilot: Powered by main battery via ESC power module
```

---

## Network Configuration

### Tailscale Setup (Primary)

Tailscale provides secure connectivity over any network.

**On Jetson (should already be configured):**

```bash
# Check Tailscale status
tailscale status

# If not connected:
sudo tailscale up --authkey=<YOUR_AUTH_KEY>
```

**On Ground Station Laptop:**

```bash
# Install Tailscale: https://tailscale.com/download
# Login with same account as Jetson
tailscale up
```

**Get Jetson IP:**

```bash
# From Jetson:
tailscale ip -4

# Expected: 100.x.x.x address
```

### WiFi Fallback (Competition Venue)

If Tailscale unavailable, use direct WiFi:

```bash
# On Jetson - connect to competition WiFi:
nmcli device wifi connect "VENUE_SSID" password "PASSWORD"

# Get local IP:
ip addr show wlan0 | grep inet
```

---

## Jetson Orin Nano Setup

### Quick Start (2-3 minutes)

SSH into the Jetson and run:

```bash
# 1. Update NOMAD code
cd ~/NOMAD
git pull origin main

# 2. Start all services
./scripts/start_nomad_full.sh
```

### Manual Start (if script fails)

```bash
# 1. Start MAVLink Router (connect telemetry to Mission Planner)
mavlink-routerd -e <GROUND_STATION_IP>:14550 -e 127.0.0.1:14551 /dev/ttyACM0 &

# 2. Start Edge Core API
cd ~/NOMAD
python3 -m edge_core.main &

# 3. Start Isaac ROS + ZED (Task 2 only)
~/NOMAD/scripts/start_isaac_ros_auto.sh start
```

### Service Status Check

```bash
# Check all services via API:
curl http://localhost:8000/api/services/status | jq

# Or individually:
ps aux | grep mavlink
ps aux | grep edge_core
docker ps  # Check Isaac ROS container
```

---

## Ground Station Setup

### Mission Planner Installation

1. **Install Mission Planner** (if not installed):
   - Download from: https://ardupilot.org/planner/docs/mission-planner-installation.html

2. **Install NOMAD Plugin**:
   ```powershell
   # Copy plugin DLL to Mission Planner plugins folder
   Copy-Item "NOMAD\mission_planner\src\bin\Release\NOMADPlugin.dll" `
     "C:\Program Files (x86)\Mission Planner\plugins\"
   ```

3. **Install LibVLC** (for video streaming):
   ```powershell
   cd NOMAD\mission_planner\packaging
   .\fetch-libvlc.ps1
   .\copy-libvlc.ps1
   ```

### Connect to Drone

1. Open Mission Planner
2. **Connection Type**: UDP
3. **Port**: 14550
4. Click **CONNECT**

If using Tailscale:
- Ensure both devices are on the same Tailscale network
- MAVLink router automatically routes to your IP

### Open NOMAD Plugin

1. In Mission Planner: **View** -> **NOMAD Control Panel**
2. Verify connection status shows green
3. Check Jetson health metrics

---

## System Verification

### Pre-Flight Checks

Run these checks before each flight:

```bash
# From ground station - verify all endpoints:

# 1. Basic connectivity
curl http://<JETSON_TAILSCALE_IP>:8000/health

# 2. MAVLink status
# Check Mission Planner shows "Connected"

# 3. VIO status (Task 2)
curl http://<JETSON_TAILSCALE_IP>:8000/api/vio/status

# 4. Camera status
curl http://<JETSON_TAILSCALE_IP>:8000/api/camera/status

# 5. Isaac ROS status (Task 2)
curl http://<JETSON_TAILSCALE_IP>:8000/api/isaac/status
```

### Expected Responses

**Healthy System:**
```json
{
  "status": "ok",
  "connected": true,
  "gps_fix": true,  // Outdoors only
  "cpu_temp": 45,
  "gpu_temp": 42,
  "memory_used_pct": 35,
  "throttled": false
}
```

**VIO Ready (Task 2):**
```json
{
  "health": "healthy",
  "tracking_confidence": 1.0,
  "position_valid": true,
  "source": "isaac_ros"
}
```

---

## Task 1 (Outdoor Recon) Setup

Task 1 uses GPS-based navigation. VIO is not required.

### Configuration

1. **In Mission Planner NOMAD Panel:**
   - Select **Task 1: Outdoor Recon**
   - Load waypoint mission file

2. **Verify GPS Lock:**
   - Mission Planner shows 3D GPS fix
   - HDOP < 1.5
   - Satellite count > 10

### Mission Upload

```bash
# Via Mission Planner:
# 1. Plan -> Load WP File
# 2. Select mission file
# 3. Write WPs
```

### Capture Points

At each target:
1. Click **Capture** in NOMAD panel
2. Verify capture logged:
   ```bash
   curl http://<IP>:8000/api/task/1/capture
   ```

---

## Task 2 (Indoor Extinguish) Setup

Task 2 uses VIO (Visual-Inertial Odometry) for GPS-denied navigation.

### Start Isaac ROS + ZED

**Option 1: Via API**
```bash
curl -X POST http://<IP>:8000/api/isaac/start
```

**Option 2: Via NOMAD Plugin**
- In Service Control Panel, click **Start** for Isaac ROS

**Option 3: Via SSH**
```bash
~/NOMAD/scripts/start_isaac_ros_auto.sh start
```

### Verify VIO Active

```bash
# Check VIO pose updates:
curl http://<IP>:8000/api/vio/pose

# Should show x, y, z coordinates updating
```

### Reset Origin Before Flight

**Important:** Reset VIO origin at takeoff position!

```bash
curl -X POST http://<IP>:8000/api/vio/reset_origin
```

Or click **Reset Origin** in NOMAD panel.

### Monitor During Flight

- Watch VIO trajectory in NOMAD dashboard
- Confidence should stay > 0.8
- If tracking lost, hover and wait for recovery

### Target Tracking

When target is sprayed:
```bash
# Register hit in exclusion map:
curl -X POST http://<IP>:8000/api/task/2/target_hit \
  -H "Content-Type: application/json" \
  -d '{"x": 1.5, "y": 2.3, "z": 0.5}'
```

Targets in exclusion map won't be re-engaged.

---

## Troubleshooting

### "Connection Refused" to Edge Core

```bash
# Check if edge_core is running:
ssh mad@<IP> "ps aux | grep edge_core"

# If not running, start it:
ssh mad@<IP> "cd ~/NOMAD && python3 -m edge_core.main &"
```

### MAVLink Not Connecting

```bash
# Check USB device:
ssh mad@<IP> "ls -la /dev/ttyACM*"
# Should show /dev/ttyACM0

# Restart MAVLink router:
ssh mad@<IP> "pkill mavlink-routerd"
ssh mad@<IP> "mavlink-routerd -e <YOUR_IP>:14550 -e 127.0.0.1:14551 /dev/ttyACM0 &"
```

### ZED Camera Not Detected

```bash
# Check USB connection:
ssh mad@<IP> "lsusb | grep -i stereo"

# Should show: Stereolabs ZED 2i

# If not shown:
# - Check USB cable (must be USB 3.0 capable)
# - Try different USB port
# - Reboot Jetson
```

### VIO Tracking Lost

1. Hold position and hover
2. Ensure good lighting
3. Check for occlusions (hand in front of camera)
4. Reset origin if needed:
   ```bash
   curl -X POST http://<IP>:8000/api/vio/reset_origin
   ```

### High CPU/GPU Temperature

```bash
# Check temps:
curl http://<IP>:8000/health/detailed

# If > 75C:
# - Reduce processing (lower FPS)
# - Improve airflow
# - Consider heatsink/fan
```

### Isaac ROS Container Won't Start

```bash
# Check Docker status:
ssh mad@<IP> "docker ps -a | grep nomad"

# View logs:
ssh mad@<IP> "docker logs nomad_isaac_ros --tail 50"

# Remove and restart:
ssh mad@<IP> "docker rm -f nomad_isaac_ros"
ssh mad@<IP> "~/NOMAD/scripts/start_isaac_ros_auto.sh start"
```

---

## Emergency Procedures

### Loss of VIO During Flight (Task 2)

1. **Immediately switch to STABILIZE mode** (RC switch)
2. Hold position visually
3. Slowly navigate to exit using visual references
4. Do NOT rely on automated waypoints

### Loss of Communication

1. MAVLink failsafe activates (configured in CubePilot):
   - Default: RTL (Return to Launch) after 30s
2. If RTL not possible (indoors), switch to STABILIZE and land manually

### Thermal Throttling

If Jetson enters thermal throttling:
1. Vision processing will slow down
2. Land immediately if VIO confidence drops < 0.5
3. Allow 5 minutes cooldown before next flight

### Emergency Shutdown

```bash
# Stop all services and shutdown Jetson:
ssh mad@<IP> "sudo shutdown now"
```

Or pull the power if network unreachable.

---

## Quick Reference Card

Print this for competition day:

```
+------------------------------------------+
|          NOMAD QUICK REFERENCE           |
+------------------------------------------+
| Jetson IP: 100.85.121.98 (Tailscale)    |
| API Port: 8000                           |
| MAVLink: UDP 14550                       |
+------------------------------------------+
| START ALL:                               |
| ssh mad@IP "~/NOMAD/scripts/             |
|   start_nomad_full.sh"                   |
+------------------------------------------+
| CHECK HEALTH:                            |
| curl http://IP:8000/health               |
+------------------------------------------+
| START ISAAC ROS:                         |
| curl -X POST http://IP:8000/             |
|   api/isaac/start                        |
+------------------------------------------+
| RESET VIO ORIGIN:                        |
| curl -X POST http://IP:8000/             |
|   api/vio/reset_origin                   |
+------------------------------------------+
| EMERGENCY: Switch to STABILIZE + LAND    |
+------------------------------------------+
```

---

## Appendix: Full API Reference

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/health` | GET | System health overview |
| `/health/detailed` | GET | Full Jetson metrics |
| `/status` | GET | Complete system state |
| `/api/vio/status` | GET | VIO health and stats |
| `/api/vio/pose` | GET | Current VIO position |
| `/api/vio/trajectory` | GET | Trajectory history |
| `/api/vio/reset_origin` | POST | Reset VIO origin |
| `/api/isaac/status` | GET | Isaac ROS status |
| `/api/isaac/start` | POST | Start Isaac ROS |
| `/api/isaac/stop` | POST | Stop Isaac ROS |
| `/api/camera/status` | GET | ZED camera status |
| `/api/task/1/capture` | POST | Capture Task 1 point |
| `/api/task/2/target_hit` | POST | Register target hit |
| `/api/task/2/exclusion_map` | GET | Get exclusion map |
| `/api/services/status` | GET | All service statuses |

---

*Last updated: January 26, 2026*
*NOMAD - MAD Team AEAC 2026*
