# NOMAD Jetson Orin Nano Deployment Guide

**Last Updated:** January 21, 2026  
**Target:** NVIDIA Jetson Orin Nano (JetPack 5.x / Ubuntu 22.04)

---

## System Information

| Property | Value |
|----------|-------|
| **Hostname** | ubuntu |
| **Username** | mad |
| **OS** | Ubuntu 22.04.5 LTS |
| **Kernel** | 5.15.148-tegra (aarch64) |
| **Tailscale IP** | 100.85.121.98 |
| **Ground Station IP** | 100.76.127.17 |

---

## Installation Paths

### NOMAD Edge Core
```
/home/mad/NOMAD/
├── edge_core/           # Python FastAPI server
│   ├── main.py          # Entry point
│   ├── api.py           # REST endpoints
│   ├── mavlink_interface.py
│   ├── state.py
│   └── ...
├── config/
│   └── env/
│       └── jetson.env   # Environment template
├── transport/
│   └── mavlink_router/
│       └── main.conf    # MAVLink routing config
├── .env                 # Active environment config
└── start_nomad.sh       # Startup script
```

### System Services
```
/etc/mavlink-router/main.conf    # MAVLink Router config (if installed)
/etc/systemd/system/nomad.service  # NOMAD systemd service (optional)
```

### Python Dependencies
```
/home/mad/.local/lib/python3.10/site-packages/
├── fastapi/
├── uvicorn/
├── pymavlink/
├── pyzmq/
└── python-dotenv/
```

### Logs
```
/home/mad/nomad.log              # Edge Core output (when run with nohup)
```

---

## How to Start NOMAD

### Method 1: Unified Startup (Recommended)
Starts both Edge Core API and ZED Video Stream:
```bash
ssh mad@100.85.121.98
~/start_nomad_full.sh
```

### Method 2: Background Mode
```bash
ssh mad@100.85.121.98 "nohup ~/start_nomad_full.sh > /dev/null 2>&1 &"
```

### Method 3: Systemd Service (Auto-start on boot)
```bash
# Install the service
sudo cp ~/nomad.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable nomad
sudo systemctl start nomad

# Check status
sudo systemctl status nomad
```

### Method 4: Manual Start (Components Separately)
```bash
# Start Edge Core only
~/start_nomad.sh

# Start Video Stream only
~/start_zed_stream.sh
```

---

## How to Stop NOMAD

### If running in foreground:
Press `Ctrl+C`

### If running with nohup:
```bash
pkill -f "edge_core.main"
```

### If using systemd:
```bash
sudo systemctl stop nomad
```

---

## Verify NOMAD is Running

### From Jetson (localhost):
```bash
curl http://localhost:8000/health
```

### From Windows Ground Station:
```powershell
Invoke-WebRequest -Uri "http://100.85.121.98:8000/health" -UseBasicParsing | Select-Object -ExpandProperty Content
```

### Expected Response:
```json
{
  "status": "degraded",
  "connected": false,
  "gps_fix": false,
  "flight_mode": "UNKNOWN",
  "timestamp": "2026-01-21T16:54:12.784005+00:00"
}
```

**Note:** `"connected": false` is normal when no flight controller is connected.

---

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Service information |
| `/health` | GET | Quick health check with Jetson metrics |
| `/health/detailed` | GET | Full Jetson health (CPU, GPU, memory, disk, fan) |
| `/status` | GET | Current drone state (telemetry) |
| `/docs` | GET | Swagger API documentation |
| `/api/task/1/capture` | POST | Trigger Task 1 capture |
| `/api/task/2/exclusion_map` | GET | Get exclusion map |
| `/api/task/2/reset_map` | POST | Reset exclusion map |
| `/api/task/2/target_hit` | POST | Mark target as hit |
| `/api/vio/status` | GET | VIO pipeline status |
| `/api/vio/reset_origin` | POST | Reset VIO origin |
| `/api/isaac/status` | GET | Isaac ROS bridge status |
| `/api/terminal/exec` | POST | Execute terminal command |
| `/ws/state` | WS | WebSocket real-time state (10Hz) |

### Access Swagger UI:
```
http://100.85.121.98:8000/docs
```

---

## Video Streaming

### ZED 2i Camera Status
The ZED 2i camera is connected and accessible via V4L2:
- Device: `/dev/video0`
- Resolution: 1344x376 (stereo side-by-side) or 672x376 (left eye only)
- Frame Rate: 30 FPS

### Start Video Stream (UDP to Ground Station)
```bash
ssh mad@100.85.121.98
~/start_zed_stream.sh
```

### View Stream on Windows (VLC)
```
vlc udp://@:5600
```

Or create an SDP file `zed.sdp`:
```
v=0
m=video 5600 RTP/AVP 96
c=IN IP4 100.76.127.17
a=rtpmap:96 H264/90000
```
Then: `vlc zed.sdp`

### Stream Parameters
- Codec: H.264
- Bitrate: 2 Mbps
- Port: UDP 5600
- Resolution: 672x376 (left eye)
- Latency: ~100-200ms over Tailscale

---

## Configuration

### Environment Variables (`.env`)
Located at: `/home/mad/NOMAD/.env`

```bash
# Network
NOMAD_HOST=0.0.0.0
NOMAD_PORT=8000
TAILSCALE_IP=100.85.121.98
GCS_IP=100.76.127.17
GCS_PORT=14550

# Hardware
MAVLINK_UART_DEV=/dev/ttyTHS1
MAVLINK_UART_BAUD=921600

# Features
NOMAD_ENABLE_VISION=true
NOMAD_ENABLE_ISAAC_ROS=false
NOMAD_DEBUG=false
```

### To Edit Configuration:
```bash
nano ~/NOMAD/.env
# Then restart Edge Core
```

---

## Network Ports

| Port | Protocol | Service | Direction |
|------|----------|---------|-----------|
| 8000 | TCP | Edge Core API | Inbound |
| 8554 | TCP | RTSP Video | Inbound |
| 14550 | UDP | MAVLink Telemetry | Outbound to GCS |
| 22 | TCP | SSH | Inbound |

---

## Updating NOMAD

### Pull Latest Code:
```bash
cd ~/NOMAD
git pull origin main
```

### Update Dependencies:
```bash
pip3 install --user -r edge_core/requirements.txt
```

### Restart Service:
```bash
# If using nohup
pkill -f "edge_core.main"
nohup ~/start_nomad.sh > ~/nomad.log 2>&1 &

# If using systemd
sudo systemctl restart nomad
```

---

## Troubleshooting

### Edge Core Won't Start

**Check Python path:**
```bash
which python3
python3 --version  # Should be 3.10+
```

**Check dependencies:**
```bash
pip3 list | grep -E "fastapi|uvicorn|pymavlink"
```

**Check for errors:**
```bash
cd ~/NOMAD
python3 -m edge_core.main
```

### Can't Connect from Windows

**Check Tailscale:**
```bash
# On Jetson
tailscale status

# On Windows
tailscale status
ping 100.85.121.98
```

**Check firewall:**
```bash
sudo ufw status
sudo ufw allow from 100.0.0.0/8 to any port 8000
```

### MAVLink Not Working

**Check UART permissions:**
```bash
ls -la /dev/ttyTHS1
sudo usermod -aG dialout mad
# Then logout and login again
```

**Check MAVLink Router:**
```bash
sudo systemctl status mavlink-router
cat /etc/mavlink-router/main.conf
```

### View Logs

**Edge Core logs:**
```bash
cat ~/nomad.log
tail -f ~/nomad.log
```

**System logs:**
```bash
journalctl -u nomad -f
dmesg | tail -50
```

---

## Quick Reference

### SSH Access
```bash
ssh mad@100.85.121.98
# Password: Set via JETSON_SSH_PASS environment variable
# DO NOT commit passwords to version control
```

### Start NOMAD
```bash
nohup ~/start_nomad.sh > ~/nomad.log 2>&1 &
```

### Stop NOMAD
```bash
pkill -f "edge_core.main"
```

### Check Status
```bash
curl localhost:8000/health
```

### View Logs
```bash
tail -f ~/nomad.log
```

### Pull Updates
```bash
cd ~/NOMAD && git pull
```

---

## Security Notes

1. **Change the default password** when deploying to production
2. **Enable SSH key authentication** and disable password auth
3. **Keep Tailscale updated** for security patches
4. **Monitor logs** for suspicious activity

---

## Support

- **Repository:** https://github.com/YoussGm3o8/NOMAD
- **Issues:** https://github.com/YoussGm3o8/NOMAD/issues
