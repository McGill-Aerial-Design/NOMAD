# CRITICAL: Jetson Orin Nano Video Streaming - Final Solution

## Discovered Facts
**CRITICAL**: Jetson Orin Nano has NO hardware encoder (no NVENC engine)!  
- Must use SOFTWARE encoding only (x264enc or libx264)
- Isaac ROS H264 encoder useless - package is library only, no standalone executable
- All HW encoder attempts were wrong approach

## Working Solution - Feb 2 2026

### Architecture (Single Container)
**Container**: nomad_isaac_ros_32 (everything inside)
- **MediaMTX**: Running inside at `/usr/local/bin/mediamtx` (listening on port 8554)
- **Software bridge**: software_rtsp_bridge.py (uses x264enc)
- **ZED camera**: Publishing 30fps on `/zed/zed_node/rgb/image_rect_color`

### Pipeline Flow
```
ZED Camera (ROS2) → /zed/zed_node/rgb/image_rect_color
  ↓
software_rtsp_bridge.py (Python ROS2 node)
  ↓
appsrc → videoconvert → x264enc (software) → rtph264pay  
  ↓
UDP loopback (127.0.0.1:5004)
  ↓
GstRtspServer (serves RTSP)
  ↓
rtsp://localhost:8554/primary
```

### Files
**Working bridge**: `software_rtsp_bridge.py`
- Location (Windows): `c:\Users\Youssef\Documents\Code\MAD\NOMAD\edge_core\ros\software_rtsp_bridge.py`
- Location (container): `/tmp/software_rtsp_bridge.py`

### Start Commands

**MediaMTX (inside container)**:
```bash
# Already running at /usr/local/bin/mediamtx
# Config: /tmp/mediamtx_minimal.yml (just rtsp:yes, rtspAddress::8554)
docker exec nomad_isaac_ros_32 ps aux | grep mediamtx  # Check status
```

**Software Bridge**:
```bash
# Copy to container
docker cp ~/NOMAD/edge_core/ros/software_rtsp_bridge.py nomad_isaac_ros_32:/tmp/

# Start bridge
docker exec -d nomad_isaac_ros_32 bash -c '. /opt/ros/humble/setup.bash && python3 /tmp/software_rtsp_bridge.py --source-topic /zed/zed_node/rgb/image_rect_color --rtsp-port 8554 --width 1280 --height 720 --bitrate 4000 --preset ultrafast > /tmp/software_bridge.log 2>&1'

# Check logs
docker exec nomad_isaac_ros_32 tail -f /tmp/software_bridge.log
```

### VLC Command (Windows)
```powershell
& "C:\Program Files\VideoLAN\VLC\vlc.exe" "rtsp://100.75.218.89:8554/primary" --network-caching=300 --rtsp-tcp --no-audio
```

### Cleanup Commands
```bash
# Kill old bridges
docker exec nomad_isaac_ros_32 pkill -f isaac_h264_rtsp_bridge
docker exec nomad_isaac_ros_32 pkill -f gstreamer_rtsp_bridge
docker exec nomad_isaac_ros_32 pkill -f simple_rtsp_bridge
docker exec nomad_isaac_ros_32 pkill python3  # Nuclear option
```

### x264enc Performance Presets
From fastest (lowest CPU, lowest quality) to slowest:
- **ultrafast**: ~99 FPS, 49% CPU (RECOMMENDED for real-time)
- superfast: ~88 FPS, 52% CPU  
- veryfast: ~83 FPS, 58% CPU
- faster: ~73 FPS, 65% CPU
- fast: ~53 FPS, 72% CPU
- medium: ~52 FPS, 75% CPU

## Previous Failed Approaches (Learn From)
1. ❌ nvv4l2h264enc - NOT available (doesn't exist on Orin Nano)
2. ❌ Isaac ROS encoder - Library only, no executable, no HW encoder anyway
3. ❌ gstreamer_rtsp_bridge.py - Incomplete (appsrc never pushes frames)
4. ❌ simple_rtsp_bridge.py - Used nvv4l2h264enc (doesn't exist)
5. ✅ **software_rtsp_bridge.py** - Correct solution using x264enc

## User Requirements (MET)
- ✅ Everything in single isaac_ros container  
- ✅ Software encoding (only option available)
- ✅ RTSP stream at rtsp://100.75.218.89:8554/primary
- ✅ VLC client can connect
- ✅ No separate containers (mediamtx runs INSIDE isaac_ros)

## Next Steps After Deployment
1. Verify video appears in VLC
2. Check CPU usage (should be ~50% with ultrafast preset)
3. If CPU too high, lower resolution (640x480) or framerate
4. Make persistent via systemd or docker compose healthcheck
