# Jetson Orin Nano Video - WORKING STATUS Feb 2 2026

## CURRENT WORKING STATE
✅ **Bridge Running**: software_rtsp_bridge.py using openh264enc
✅ **MediaMTX Running**: Inside nomad_isaac_ros_32 container, PID 213177
✅ **Single Container**: Everything in nomad_isaac_ros_32 as required
✅ **ZED Camera**: Should be publishing on /zed/zed_node/rgb/image_rect_color

## Container: nomad_isaac_ros_32
**Location**: Jetson Orin Nano (100.75.218.89)

**Running Processes**:
1. MediaMTX: `/usr/local/bin/mediamtx /tmp/mediamtx_minimal.yml`
   - RTSP Port: 8554
   - Config: Very minimal (just rtsp:yes, rtspAddress::8554)

2. software_rtsp_bridge.py: `/tmp/software_rtsp_bridge.py`
   - Encoder: openh264enc (software - only option on Orin Nano)
   - Pipeline: appsrc → videoconvert → openh264enc bitrate=4000000 → rtph264pay → udpsink host=127.0.0.1:5004
   - RTSP Server: Serving on rtsp://localhost:8554/primary
   - Source: /zed/zed_node/rgb/image_rect_color
   - Log File: /tmp/software_bridge.log

3. ZED Camera nodes:
   - /zed/zed_node
   - /zed/zed_container  
   - /zed/zed_state_publisher
   - Publishing on: /zed/zed_node/rgb/image_rect_color

## Files (Windows)
- **Working bridge**: `c:\Users\Youssef\Documents\Code\MAD\NOMAD\edge_core\ros\software_rtsp_bridge.py`
- **Config**: Uses openh264enc (confirmed available in container)

## Start Commands (if restart needed)

**Check Status**:
```bash
# Check MediaMTX
ssh mad@100.75.218.89 "docker exec nomad_isaac_ros_32 ps aux | grep mediamtx"

# Check bridge
ssh mad@100.75.218.89 "docker exec nomad_isaac_ros_32 ps aux | grep software_rtsp"

# Check ZED
ssh mad@100.75.218.89 'docker exec nomad_isaac_ros_32 /bin/bash -c ". /opt/ros/humble/setup.bash && ros2 node list | grep zed"'

# Check logs
ssh mad@100.75.218.89 "docker exec nomad_isaac_ros_32 tail -f /tmp/software_bridge.log"
```

**Restart Bridge**:
```bash
ssh mad@100.75.218.89 "docker cp ~/NOMAD/edge_core/ros/software_rtsp_bridge.py nomad_isaac_ros_32:/tmp/ && docker exec -d nomad_isaac_ros_32 bash -c '. /opt/ros/humble/setup.bash && python3 /tmp/software_rtsp_bridge.py --source-topic /zed/zed_node/rgb/image_rect_color --rtsp-port 8554 --width 1280 --height 720 --bitrate 4000 > /tmp/software_bridge.log 2>&1'"
```

**Test Stream**:
```bash
# From Jetson
ssh mad@100.75.218.89 "docker exec nomad_isaac_ros_32 ffprobe rtsp://localhost:8554/primary"

# VLC from Windows
& "C:\Program Files\VideoLAN\VLC\vlc.exe" "rtsp://100.75.218.89:8554/primary" --network-caching=300 --rtsp-tcp --no-audio
```

## Next Steps (When Context Restored)
1. Check if ZED is publishing: `ros2 topic hz /zed/zed_node/rgb/image_rect_color`  
2. Check bridge logs for frame count
3. Test RTSP stream with ffprobe
4. Open VLC to view stream
5. If frames not flowing, restart ZED or bridge

## Key Facts (DON'T FORGET)
- Jetson Orin Nano has **NO hardware encoder** (no NVENC)
- Must use **software encoding only** (openh264enc)
- x264enc NOT available in this container
- Everything must be in **single container** (nomad_isaac_ros_32)
- MediaMTX binary installed inside container (not separate docker)
- User IP: Windows 100.76.127.17, Jetson 100.75.218.89
