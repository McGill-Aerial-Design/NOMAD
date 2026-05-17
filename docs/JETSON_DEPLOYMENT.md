# NOMAD Jetson Orin Nano Deployment Guide

> **Refactor note (2026-05):** runtime is now systemd-based. Use
> `scripts/nomad` and edit `config/nomad.env`. References below to
> `start_nomad_full.sh`, `start_isaac_ros_auto.sh`, `restart_nomad.sh`, or
> `config/env/jetson.env` are pre-refactor — see `scripts/README.md` and
> `docs/OPERATIONS_RUNBOOK.md` for the current commands.

**Last Updated:** May 9, 2026  
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
3. **Load management**: If throttling occurs, the unified `nvblox_performance.yaml` profile is tuned for safe operation

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
4. The unified nvblox profile handles both outdoor and indoor operations
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
├── edge_core/                # Python FastAPI server
│   ├── main.py               # Entry point
│   ├── api.py                # REST / WebSocket endpoints
│   ├── mavlink_interface.py  # MAVLink telemetry + commands
│   ├── nav_controller.py     # Velocity / position command routing
│   ├── servo_controller.py   # Camera tilt / water shooter PWM
│   ├── spray_controller.py   # Fire-extinguisher spray control
│   ├── operational_mode.py   # Operational mode state machine
│   ├── video_stream_manager.py # Video bridge management
│   ├── isaac_ros_bridge.py   # Isaac ROS / nvblox lifecycle
│   ├── health_monitor.py     # Jetson hardware monitoring
│   ├── state.py              # Global state manager
│   └── ...
├── config/
│   └── env/
│       └── jetson.env        # Environment template
├── transport/
│   └── mavlink_router/
│       └── main.conf         # MAVLink routing config
├── infra/
│   └── mediamtx.yml          # MediaMTX RTSP server config
├── scripts/
│   ├── run/
│   │   ├── start_nomad_full.sh    # Full system startup
│   │   ├── start_isaac_ros_auto.sh # Isaac ROS container lifecycle
│   │   └── restart_nomad.sh       # Kill-all and restart
│   ├── setup/
│   │   ├── setup_service.sh       # systemd nomad.service install
│   │   └── setup_jetson.sh        # Full Jetson initial setup
│   └── build/
│       └── build_plugin_windows.ps1 # C# plugin build
└── .env                      # Active environment config
```

### System Services
```
/etc/systemd/system/nomad.service  # NOMAD Edge Core systemd service
```

### Python Dependencies
```
/home/mad/NOMAD/venv/   # Python virtualenv (used by systemd service)
```

### Logs
```
~/nomad_logs/              # Log directory (mediamtx, mavlink, etc.)
/home/mad/nomad.log        # Edge Core output (when run with nohup)
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
Starts MAVLink Router, Edge Core (if not already running via systemd), and prepares the system:
```bash
ssh mad@100.85.121.98
cd ~/NOMAD
./scripts/run/start_nomad_full.sh
```

### Method 2: Systemd Service (Auto-start on boot)
```bash
# Install the service
sudo bash ~/NOMAD/scripts/setup/setup_service.sh

# Check status
sudo systemctl status nomad

# Edge Core auto-starts on boot once the service is installed
```

### Method 3: Manual Start (Components Separately)
```bash
# Start Edge Core only (via systemd)
sudo systemctl start nomad

# Start MAVLink Router manually
GCS_IP=100.76.127.17 mavlink-routerd -e ${GCS_IP}:14560 -e 127.0.0.1:14550 /dev/ttyACM0 &

# Start Isaac ROS (Task 2 only)
~/NOMAD/scripts/run/start_isaac_ros_auto.sh start
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

The full API is auto-documented at:
```
http://100.85.121.98:8000/docs
```

### System / Health
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Service information |
| `/health` | GET | Quick health check with Jetson metrics |
| `/health/detailed` | GET | Full Jetson health (CPU, GPU, memory, disk) |
| `/status` | GET | Current drone state (telemetry) |
| `/docs` | GET | Swagger API documentation |
| `/api/services/status` | GET | systemd / process status |

### Task 1 (Outdoor Recon)
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/task/1/target/capture` | POST | Trigger target detection + description |
| `/api/task/1/target/save` | POST | Save targets to competition .txt file |
| `/api/task/1/target/clear` | POST | Clear all captured targets |
| `/api/task/1/target/list` | GET | List captured targets |
| `/api/task/1/target/detections` | GET | Current frame detection overlay |
| `/api/task/1/target/model` | GET | Building model summary |
| `/api/task/1/capture` | POST | Legacy capture fallback |
| `/api/task/1/captures` | GET | List capture folders |
| `/api/task/1/building/corner` | POST | Save one building corner GPS |
| `/api/task/1/building/corners` | GET | List saved building corners |
| `/api/task/1/building/corners/apply` | POST | Rebuild building model from corners |

### Task 2 (Indoor VIO)
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/task/2/reset_map` | POST | Clear exclusion map |
| `/api/task/2/target_hit` | POST | Mark target as hit |
| `/api/task/2/exclusion_map` | GET | Get hit targets |

### VIO
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/vio/status` | GET | VIO pipeline status |
| `/api/vio/pose` | GET | Current position/orientation |
| `/api/vio/trajectory` | GET | Path history |
| `/api/vio/reset_origin` | POST | Reset VIO origin |
| `/api/vio/calibration` | GET | ZED calibration state |

### Navigation / Isaac ROS
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/nav/status` | GET | Navigation controller status |
| `/api/nav/velocity` | POST | Send velocity command |
| `/api/nav/stop` | POST | Emergency stop |
| `/api/isaac/status` | GET | Isaac ROS bridge status |
| `/api/isaac/start` | POST | Start Isaac ROS container |
| `/api/isaac/stop` | POST | Stop Isaac ROS container |

### Servo / Spray / Video / Terminal
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/servo/status` | GET | Cube servo and relay status |
| `/api/servo/camera/tilt?angle=N` | POST | Set camera servo angle |
| `/api/servo/shooter/trigger?duration_ms=N` | POST | Trigger water shooter Cube relay |
| `/api/spray/status` | GET | Spray controller state |
| `/api/spray/trigger` | POST | Trigger spray sequence |
| `/api/video/status` | GET | Video pipeline status |
| `/api/video/start` | POST | Start video streaming |
| `/api/terminal/commands` | GET | List whitelisted commands |
| `/api/terminal/logs` | GET | Recent command logs |

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

### Architecture
Video streaming uses **MediaMTX** (bare metal, port 8554) to serve RTSP. The video encoding pipeline runs inside the Isaac ROS container using software `openh264enc` (Orin Nano lacks NVENC).

```
ZED -> Isaac ROS -> ROS Topic -> GStreamer/openh264enc -> MediaMTX RTSP -> Network -> Mission Planner (LibVLC)
```

### Start Video Stream
Video is started automatically as part of `start_nomad_full.sh` or can be started via API:
```bash
curl -X POST http://localhost:8000/api/video/start
```

### View Stream (RTSP)
```
rtsp://100.85.121.98:8554/primary
```

In Mission Planner, the NOMAD plugin embeds a LibVLC player that connects to this RTSP URL automatically.

### Stream Parameters
- Codec: H.264 (software openh264enc)
- Bitrate: Configurable (see `infra/mediamtx.yml`)
- Protocol: RTSP (TCP)
- Port: 8554
- Latency: ~100-200ms over Tailscale

### MediaMTX Management
MediaMTX runs as a systemd user service (`mediamtx.service`) and auto-starts on boot. To manage:
```bash
systemctl --user status mediamtx
systemctl --user restart mediamtx
# Or via Edge Core API:
curl -s http://localhost:8000/api/services/status
```

---

## Configuration

### Environment Variables (`config/env/jetson.env`)
Located at: `/home/mad/NOMAD/config/env/jetson.env`

This is the primary configuration file. It contains:
- Jetson Tailscale IP (`TAILSCALE_IP=100.85.121.98`)
- Ground Station IP (`GCS_IP=100.76.127.17`)
- Home paths (`/home/mad/NOMAD/`)
- Port configuration
- Feature flags

### To Edit Configuration:
```bash
nano ~/NOMAD/config/env/jetson.env
# Then restart Edge Core
```

---

## Network Ports

| Port | Protocol | Service | Direction |
|------|----------|---------|-----------|
| 8000 | TCP | Edge Core API | Inbound |
| 8554 | TCP | RTSP Video | Inbound |
| 14560 | UDP | MAVLink LTE/Tailscale | Outbound to GCS |
| 14550 | UDP | MAVLink local/RadioMaster | Local Edge Core / ground radio |
| 14600 | UDP | MAVLink plugin router | Ground Station local merged stream |
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
cd ~/NOMAD
./venv/bin/pip install -r edge_core/requirements.txt
```

### Restart Service:
```bash
# If using systemd (recommended)
sudo systemctl restart nomad
```

---

## Troubleshooting

### Edge Core Won't Start

**Check Python path:**
```bash
which python3
python3 --version # Should be 3.10+ (JetPack 6.x ships Python 3.10)
```

**Check dependencies (in venv):**
```bash
cd ~/NOMAD
./venv/bin/pip list | grep -E "fastapi|uvicorn|pymavlink"
```

**Check for errors:**
```bash
cd ~/NOMAD
./venv/bin/python -m edge_core.main
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

**Check USB device permissions:**
```bash
ls -la /dev/ttyACM0
sudo usermod -aG dialout mad
# Then logout and login again
```

**Check MAVLink Router:**
```bash
pgrep -f mavlink-routerd
# MAVLink Router runs as a bare process, not as a systemd service
cat ~/NOMAD/transport/mavlink_router/main.conf
```

### View Logs

**Edge Core logs (systemd):**
```bash
journalctl -u nomad -f
```

**Service logs directory:**
```bash
ls ~/nomad_logs/
cat ~/nomad_logs/mediamtx.log
cat ~/nomad_logs/mavlink.log
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
cd ~/NOMAD && ./scripts/run/start_nomad_full.sh
```

### Stop NOMAD
```bash
sudo systemctl stop nomad
```

### Check Status
```bash
curl localhost:8000/health
systemctl status nomad
```

### View Logs
```bash
journalctl -u nomad -f
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
