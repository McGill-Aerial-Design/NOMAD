# Video Streaming Progress - Feb 2 2026

## CURRENT STATUS - CRITICAL

### User Requirements
- **SINGLE container only**: nomad_isaac_ros_32
- **Lowest memory/latency**: Must use Jetson HW encoder
- **NO fallbacks**: One working system
- **Target**: rtsp://100.75.218.89:8554/primary
- **VLC client**: Windows 100.76.127.17

### System State
- ZED camera: 30fps on `/zed/zed_node/rgb/image_rect_color`
- Containers stopped: nomad-mediamtx, nomad-jetson-video
- VLC: Connecting but BLACK SCREEN (previous attempts)

### Discovered Facts
1. **nvv4l2h264enc NOT available** in isaac_ros container
2. **Only openh264enc available** (software, NOT acceptable for user)
3. **isaac_ros_h264_encoder package IS available** (This is the HW encoder!)

### Solution Direction
Use Isaac ROS H264 encoder node + minimal RTSP server:
1. Isaac ROS encoder subscribes to RGB topic, outputs H264 compressed
2. Simple RTSP server takes H264 topic, serves via RTSP
3. This is HW-accelerated, low latency, single container

### Files
- simple_rtsp_bridge.py: Failed (nvv4l2h264enc not found)
- isaac_h264_rtsp_bridge.py: Uses encoder correctly but needs mediamtx
- gstreamer_rtsp_bridge.py: Incomplete (doesn't push frames)

### Next Steps
1. Check Isaac H264 encoder output topic format
2. Create minimal RTSP server for H264 topic
3. OR modify isaac_h264_rtsp_bridge.py to act as server instead of client

### Key Commands
```bash
# Check running encoder
docker exec nomad_isaac_ros_32 bash -c 'source /opt/ros/humble/setup.bash && ros2 node list | grep encoder'

# Check H264 topics
docker exec nomad_isaac_ros_32 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list | grep compressed'
```
