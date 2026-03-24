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

## Resource Requirements

### Hardware Specifications

NOMAD is optimized for the **NVIDIA Jetson Orin Nano 8GB** with the following characteristics:

| Component | Spec | Notes |
|-----------|------|-------|
| **CPU** | 8-core ARM Cortex-A78 (6 cores allocated to Isaac ROS container) | 2 cores reserved for host OS |
| **GPU** | 128-core NVIDIA CUDA (Orin architecture) | Shared with Host; nvblox, ZED SDK use GPU |
| **Memory** | 8GB unified DRAM (shared with host) | 4GB container limit, 4GB host |
| **Storage** | 64GB eMMC (suggested) | Typical: 20GB NOMAD code + ROS, 40GB free |
| **Thermal** | Fanless or passive | Throttles above 80°C; shuts down ~85°C |

### Container Resource Allocation

The Isaac ROS container is configured with hard limits:

```
--memory 4g          # Conservative 50% of Orin Nano 8GB
--cpus 6             # 75% of 8 cores (leave 2 for host)
--shm-size 1g        # Bounded shared memory for ROS2 IPC
```

**Rationale:**
- **Memory**: nvblox + ZED + ROS2 typically consume 2-3GB. 4GB limit prevents container from consuming all DRAM and starving the host OS (MediaMTX, Edge Core, system daemons).
- **CPU**: nvblox computation is parallelized; 6 cores sufficient. Leaves 2 cores for host scheduling and I/O.
- **Shared Memory**: Default /dev/shm is 8GB (50% of DRAM); reduced to 1GB to prevent uncontrolled growth from dropped ROS2 zero-copy messages. Monitor with `df /dev/shm`.

### Power Mode Requirements

nvblox requires sustained GPU performance. The following Jetson power modes are recommended:

| Mode | Command | Max TDP | Min TDP | Performance | Recommended? |
|------|---------|---------|---------|-------------|--------------|
| **Jetson Clocks (Max)** | `sudo jetson_clocks` | 25W | 5W (when idle) | Maximum sustained | **BEST** |
| **25W Balanced** | `sudo nvpmodel -m 2` | 25W | 10W | Good (15W nominal) | **GOOD** |
| **15W** | `sudo nvpmodel -m 0` | 15W | 5W | Limited (moderate throttle) | **MINIMUM** |
| **5W** | `sudo nvpmodel -m 1` | 5W | 2W | Very limited (high throttle) | **NOT RECOMMENDED** |

**Minimum Requirement**: STeam performance at **15W** or higher. Below 15W, nvblox integrates depth slowly and may skip frames under load.

**Recommended**: Run **`sudo jetson_clocks`** for outdoor Task 1 (GPS) and sustained indoor Task 2 (VIO) missions.

**Verify Current Mode**:
```bash
# Check current mode (0=5W, 1=15W, 2=25W)
sudo nvpmodel -q

# Set to 25W
sudo nvpmodel -m 2

# Enable sustained max performance
sudo jetson_clocks

# Check frequencies
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq
```

### Thermal Management

Jetson **throttles CPU/GPU frequencies** if thermal sensors exceed thresholds:

| Temp Region | Behavior | Impact |
|-------------|----------|--------|
| < 70°C | Optimal | No throttling |
| 70-80°C | Alert | Minor frequency throttle (1-5%) |
| 80-85°C | Critical | Major frequency throttle (20-50%) |
| > 85°C | Shutdown | Device will power down to protect hardware |

**Monitor Temperature**:
```bash
# Real-time monitoring (requires jtop, see below)
jtop

# Check sensor values directly
cat /sys/class/hwmon/hwmon*/temp*_input | head -5

# View thermal zone state
cat /sys/class/thermal/thermal_zone*/temp | awk '{temp=$1/1000; print temp "°C"}'
```

**Thermal Targets**:
- **Outdoor Task 1**: 50-70°C (direct sunlight may increase)
- **Indoor Task 2**: 55-75°C (expected due to sustained nvblox load)
- **Idle**: < 50°C

**Cooling Strategies**:
1. **Active cooling**: Add a 40x40mm heatsink fan (5-10W additional power)
2. **Passive cooling**: Ensure airflow around Jetson enclosure
3. **Load management**: Reduce nvblox update rates if consistent > 80°C
4. **Operational mode**: Use `nvblox_safe.yaml` if Jetson consistently throttles

### Memory Layout on Orin Nano 8GB

```
System Boot:
  Host OS (Ubuntu):   1-1.5 GB (kernel, systemd services)
  Available to Apps:  6.5-7 GB

When Isaac ROS container starts:
  Container (limit):     4.0 GB (nvblox, ZED, ROS2)
  Host remaining:        3.5-4.0 GB (Edge Core, MediaMTX, system)
  /dev/shm (IPC):        1.0 GB (bounded by --shm-size)

Under Heavy Load (all services):
  ZED capture:           100-200 MB
  nvblox mesh:           800 MB - 1.2 GB (depending on voxel_size)
  ROS2 buffers:          200-500 MB
  Remaining:             800 MB - 1.5 GB (buffer)
```

**If OOM Errors Occur**:
1. Stop Isaac ROS container: `./start_isaac_ros_auto.sh stop`
2. Check memory: `free -h`
3. Kill background processes: `ps aux | grep -v "mad\|root\|[" | awk '{print $2}' | xargs kill 2>/dev/null || true`
4. Switch to `nvblox_safe.yaml` for lower memory footprint
5. Restart: `./start_isaac_ros_auto.sh start`

---

## Verify Resource Configuration

### Check Container Limits
```bash
# From host
docker stats nomad_isaac_ros --no-stream

# Output should show memory limit of ~4GB
CONTAINER      CPU %      MEM USAGE / LIMIT
nomad_isaac_ros 150%      2.3G / 4G
```

### Monitor During Operation
```bash
# Open a second SSH session
ssh mad@100.85.121.98

# Watch real-time metrics
watch -n 1 'docker stats nomad_isaac_ros --no-stream'

# Or use jtop if installed
jtop
```

### Stress Test (Optional)
To verify resource limits are enforced:
```bash
# Start container
./start_isaac_ros_auto.sh start

# Inside container, allocate memory to test limit
docker exec nomad_isaac_ros python3 << 'EOF'
import numpy as np
arrays = []
try:
    for i in range(10):
        arr = np.zeros((500, 500, 500), dtype=np.float32)  # 500 MB each
        arrays.append(arr)
        print(f"Allocated {(i+1)*500} MB")
except MemoryError:
    print("Container memory limit reached (expected at ~4GB)")
EOF
```

---

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

## Configuration Profiles

### nvblox Performance Profile
**File**: `config/nvblox_performance.yaml` (Default)

Standard aggressive configuration optimized for outdoor Task 1 and well-cooled indoor Task 2 missions:
- **Voxel Size**: 15cm (balanced resolution/speed)
- **Update Rate**: 30Hz depth integration
- **Memory**: 1.5-2 GB peak
- **Power Mode**: Jetson Clocks or 25W recommended
- **Thermal Limit**: Designed for sustained ~75°C

### nvblox Safe Profile
**File**: `config/nvblox_safe.yaml` (NEW - NV-014)

Conservative configuration for guaranteed operation under thermal or power constraints:
- **Voxel Size**: 10cm (coarser, but sufficient for 1m corridors)
- **Update Rate**: 2Hz depth integration
- **Memory**: 600-800 MB peak
- **Power Mode**: Works even at 15W mode
- **Thermal Limit**: Designed for sustained ~80°C (thermal throttling acceptable)

**Switch to Safe Profile**:

Inside the container, copy the safe config:
```bash
docker exec nomad_isaac_ros bash -c "cp /workspaces/isaac_ros-dev/config/nvblox_safe.yaml /workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml"
```

**When to Use Safe Profile**:
- Sustained Jetson temperature > 80°C despite cooling
- Available power limited to 15W mode
- Field operation with passive cooling only
- Mission does not require high-resolution obstacle detection
- Memory pressure (< 2GB available)

For detailed setup and comparison, see:
- Resource Requirements (above) - container limits and power modes
- docs/ISAAC_ROS_NVBLOX_SETUP.md - full nvblox build details
- config/nvblox_safe.yaml - configuration parameter reference

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
