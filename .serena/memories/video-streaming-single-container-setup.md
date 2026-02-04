# Video Streaming - Single Container Setup

## User Requirements (CRITICAL)
- **Everything in SINGLE container**: nomad_isaac_ros_32 ONLY
- **NO separate containers**: no mediamtx, no jetson-video
- **Lowest memory/latency**: Use Jetson HW encoder (nvv4l2h264enc)
- **NO fallbacks**: One working system only
- **Target stream**: rtsp://100.75.218.89:8554/primary
- **VLC client**: Windows 100.76.127.17 via Tailscale

## Current System State
- **Container**: nomad_isaac_ros_32 (running)
- **ZED camera**: Publishing 28-30fps on `/zed/zed_node/rgb/image_rect_color`
- **Stopped**: nomad-mediamtx, nomad-jetson-video (per user request)
- **Issue**: VLC connects but shows BLACK SCREEN

## Previous Solutions (FAILED)
1. **gstreamer_rtsp_bridge.py**: Acts as RTSP server BUT incomplete (appsrc=None, doesn't push frames - line ~130 just has `pass`)
2. **isaac_h264_rtsp_bridge.py**: Complete but needs external mediamtx server (user rejected)

## Current Solution: simple_rtsp_bridge.py
- **Location (local)**: `c:\Users\Youssef\Documents\Code\MAD\NOMAD\edge_core\ros\simple_rtsp_bridge.py`
- **Location (container)**: `/tmp/simple_rtsp_bridge.py`
- **Log file**: `/tmp/simple_bridge.log`

### Architecture
```
ROS2 Image Topic (/zed/zed_node/rgb/image_rect_color)
  ↓
appsrc (actually pushes frames!)
  ↓  
nvv4l2h264enc (Jetson HW encoder)
  ↓
rtph264pay
  ↓
udpsink (localhost:5000)
  ↓
RTSP Server (udpsrc port 5000 → rtph264depay → rtph264pay)
  ↓
RTSP clients connect to rtsp://localhost:8554/primary
```

### Key Features
- **Acts as RTSP server**: No mediamtx needed
- **Uses HW encoder**: nvv4l2h264enc for low CPU/memory
- **Actually pushes frames**: Fixed the appsrc issue
- **UDP loopback**: Connects pipeline to RTSP factory

### Start Command
```bash
docker exec -d nomad_isaac_ros_32 bash -c '
  source /opt/ros/humble/setup.bash && 
  python3 /tmp/simple_rtsp_bridge.py \
    --source-topic /zed/zed_node/rgb/image_rect_color \
    --rtsp-port 8554 \
    --width 1280 \
    --height 720 \
    --bitrate 4000000 \
  > /tmp/simple_bridge.log 2>&1'
```

### Check Status
```bash
# Check logs
docker exec nomad_isaac_ros_32 tail -f /tmp/simple_bridge.log

# Check if running
docker exec nomad_isaac_ros_32 ps aux | grep simple_rtsp_bridge

# Test locally
ssh mad@100.75.218.89 "ffprobe rtsp://localhost:8554/primary"
```

### VLC Test
```powershell
& "C:\Program Files\VideoLAN\VLC\vlc.exe" "rtsp://100.75.218.89:8554/primary" --network-caching=300 --rtsp-tcp --no-audio
```

## History
- 2026-02-02: Mediamtx docker issues fixed (YAML parsing, auth config, health check removed)
- 2026-02-02: User requested consolidation to single isaac_ros container
- 2026-02-02: Discovered gstreamer_rtsp_bridge.py incomplete
- 2026-02-02: Created simple_rtsp_bridge.py as minimal working solution
- 2026-02-02: Fixed typo in GStreamer import ('Gst Rtsp Server' → 'GstRtspServer')
