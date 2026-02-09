# NOMAD Jetson Hardware Checklist

## Current System Status (Updated: Feb 9, 2026)

### ✅ Jetson Orin Nano Setup Complete
- **OS**: Ubuntu 22.04 LTS (L4T R36.4)
- **JetPack**: 6.2
- **Storage**: 456GB NVMe SSD, 408GB free
- **Network**: Tailscale VPN (100.85.121.98)
- **Python**: venv with all dependencies
- **Edge Core**: Running as systemd service on port 8000

### ✅ Software Installed
- Docker 29.2.1 with nvidia runtime
- ZED SDK 4.2 for L4T 36.4
- Tailscale
- CUDA 12.6, drivers 540.4.0
- All Python packages (pyserial, piexif, etc.)

### ⚠️ Pending Hardware Connection
- [ ] **ZED2i Camera** - USB3 connection
- [ ] **Cube Orange Flight Controller** - USB connection (/dev/ttyACM0)
- [ ] **Camera Tilt Servo** - GPIO PWM (gpiochip0/line85)
- [ ] **Water Shooter GPIO** - GPIO output (chip0/line50)

### ⚠️ Pending Software Setup (Requires Hardware)
- [ ] **Isaac ROS workspace** - Needs NVIDIA official setup first
- [ ] **Isaac ROS Docker container** - Blocked by base image GPG key issue
- [ ] **MAVROS connection** - Needs Cube Orange connected
- [ ] **VIO pipeline** - Needs ZED camera connected

## Pre-Flight Hardware Checklist

Before powering on for competition/testing:

### Physical Connections
- [ ] ZED2i camera connected to Jetson USB3 port
- [ ] Cube Orange connected via USB (check `/dev/ttyACM0`)
- [ ] Camera tilt servo connected to GPIO 85
- [ ] Water shooter connected to GPIO 50
- [ ] Power supply connected and stable
- [ ] Jetson mounted securely on drone frame

### Software Verification Commands

```bash
# Check ZED camera detection
lsusb | grep -i stereolabs
/usr/local/zed/tools/ZED_Explorer  # Visual test

# Check Cube Orange
ls /dev/ttyACM0
# Should see MAVLink heartbeat in Edge Core logs

# Check Edge Core service
systemctl status nomad
curl http://localhost:8000/health/detailed

# Check Docker containers
docker ps
# Should see nomad-mediamtx running

# Check Tailscale connection
tailscale status
ping 100.76.127.17  # GCS
```

### Network Checklist
- [ ] Tailscale connected (100.85.121.98)
- [ ] GCS reachable (100.76.127.17)
- [ ] Internet available (for NTP, updates)
- [ ] MAVLink telemetry flowing to GCS

### API Endpoint Verification

Run from Windows GCS:

```powershell
# Basic health
Invoke-WebRequest -Uri 'http://100.85.121.98:8000/health'

# Full metrics
Invoke-WebRequest -Uri 'http://100.85.121.98:8000/health/detailed'

# Network status
Invoke-WebRequest -Uri 'http://100.85.121.98:8000/network/status'

# Servo status (after hardware connected)
Invoke-WebRequest -Uri 'http://100.85.121.98:8000/api/servo/status'

# MAVLink status (after FC connected)
Invoke-WebRequest -Uri 'http://100.85.121.98:8000/status'
```

## When Isaac ROS is Ready

### Prerequisites
1. ZED camera physically connected and detected
2. Follow NVIDIA's official Isaac ROS setup: https://nvidia-isaac-ros.github.io/getting_started/index.html
3. Complete steps 1-7 of ZED Setup Tutorial

### Then Use Our Guide
See [docs/ISAAC_ROS_ZED_SETUP.md](ISAAC_ROS_ZED_SETUP.md) for:
- Automated dependency installation script
- Launch commands for nvblox
- Troubleshooting common issues

## Troubleshooting

### ZED Camera Not Detected
```bash
lsusb | grep Stereolabs
# If not found, check USB3 connection

dmesg | tail -20
# Check for USB enumeration errors
```

### Cube Orange Not Found
```bash
ls /dev/ttyACM*
# Should see /dev/ttyACM0

# Check permissions
sudo usermod -a -G dialout $USER
# Logout/login required after this
```

### Edge Core Not Starting
```bash
journalctl -u nomad --no-pager -n 50
# Check for errors

# Manual test
cd /home/mad/NOMAD
/home/mad/NOMAD/venv/bin/python -m edge_core.main
```

### Docker Issues
```bash
docker ps -a
# Check container status

docker logs nomad-mediamtx
# Check MediaMTX logs

# Rebuild if needed
cd /home/mad/NOMAD
docker compose build jetson-video-stream
```

## Competition Day Quick Start

```bash
# 1. Power on Jetson
# 2. SSH in
ssh mad@100.85.121.98

# 3. Verify all systems
systemctl status nomad
docker ps
tailscale status

# 4. Check hardware
lsusb | grep -i "stereolabs\|cube"
curl -s http://localhost:8000/health/detailed | python3 -m json.tool

# 5. Launch video streaming (if not auto-started)
docker compose up -d jetson-video-stream

# 6. Monitor logs
journalctl -u nomad -f
```

## Quick Reference

| Component | Port | Check Command |
|-----------|------|---------------|
| Edge Core | 8000 | `curl localhost:8000/health` |
| MediaMTX RTSP | 8554 | `docker logs nomad-mediamtx` |
| MAVLink | 14550 | Check nomad logs for heartbeat |
| Tailscale | - | `tailscale status` |

## Contact for Issues

See [AGENTS.md](../AGENTS.md) for full system architecture and troubleshooting.

---

**Last Updated:** February 9, 2026  
**Jetson IP:** 100.85.121.98  
**GCS IP:** 100.76.127.17
