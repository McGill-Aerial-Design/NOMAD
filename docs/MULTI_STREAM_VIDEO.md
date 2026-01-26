# NOMAD Multi-Stream Video System

This system provides multiple simultaneous video streams from the ZED camera to Mission Planner.

## Available Streams

The system publishes the following streams to MediaMTX:

| Stream Name | ROS Topic | Description | Port |
|------------|-----------|-------------|------|
| `zed_rgb` | `/zed/zed_node/rgb/image_rect_color` | Main RGB color feed (rectified) | 9999 |
| `zed_left` | `/zed/zed_node/left/image_rect_color` | Left camera only (rectified) | 10000 |
| `zed_right` | `/zed/zed_node/right/image_rect_color` | Right camera only (rectified) | 10001 |

## Architecture

```
ZED Camera → Isaac ROS → ROS Topics → Video Bridges → FFmpeg → MediaMTX → RTSP
```

### Components

1. **Isaac ROS Container**: Runs ZED ROS2 wrapper publishing multiple topics
2. **Video Bridges** (Python): Subscribe to ROS topics, send raw frames via TCP
3. **FFmpeg Encoders**: Receive raw frames, encode with H.264, publish to MediaMTX
4. **MediaMTX**: RTSP server distributing streams to Mission Planner

## Usage

### Start All Streams

```bash
~/NOMAD/scripts/start_multi_video.sh start
```

### Check Status

```bash
~/NOMAD/scripts/start_multi_video.sh status
```

### Stop All Streams

```bash
~/NOMAD/scripts/start_multi_video.sh stop
```

### Restart Streams

```bash
~/NOMAD/scripts/start_multi_video.sh restart
```

## Integration with Mission Planner

The Mission Planner plugin includes a stream selector dropdown that:

1. Shows all available streams from MediaMTX
2. Displays live status: `[LIVE]` or `[--]`
3. Shows codec info (H264)
4. Allows switching between streams dynamically
5. Includes a refresh button (...) to query MediaMTX for current streams

### Refreshing Available Streams

Click the "..." button next to the stream selector to query MediaMTX and update the list with current streams and their status.

## Stream URLs

When streams are running, they are accessible at:

- RGB: `rtsp://jetson:8554/zed_rgb`
- Left: `rtsp://jetson:8554/zed_left`
- Right: `rtsp://jetson:8554/zed_right`

Replace `jetson` with your Tailscale IP or hostname.

## Full System Startup

The main startup script now automatically starts all streams:

```bash
~/NOMAD/scripts/start_nomad_full.sh
```

This starts:
1. MAVLink Router (telemetry)
2. MediaMTX (RTSP server)
3. Edge Core API
4. Isaac ROS + ZED camera
5. Multi-stream video system

## Configuration

Stream configuration is in `config/video_streams.json`:

```json
{
  "streams": [
    {
      "name": "zed_rgb",
      "topic": "/zed/zed_node/rgb/image_rect_color",
      "port": 9999,
      "description": "ZED RGB color (rectified)"
    },
    ...
  ]
}
```

## Adding New Streams

To add a new stream (e.g., depth):

1. Edit `config/video_streams.json` to add the new stream config
2. Edit `scripts/start_multi_video.sh`:
   - Add a new bridge start command in `start_bridges()`
   - Add a new FFmpeg encoder in `start_encoders()`
3. Restart the system

Example for depth stream:

```bash
# In start_bridges():
docker exec -d nomad_isaac_ros bash -c '
    source /opt/ros/humble/setup.bash && \
    source /workspaces/isaac_ros-dev/install/setup.bash && \
    python3 /tmp/ros_video_bridge.py \
        --topic /zed/zed_node/depth/depth_registered \
        --stream zed_depth \
        --tcp-port 10002 \
        --host localhost \
        --port 8554 \
        > /tmp/video_bridge_depth.log 2>&1
'

# In start_encoders():
nohup ffmpeg -f rawvideo -pix_fmt bgr24 -s 1280x720 -r 30 \
    -i tcp://127.0.0.1:10002 \
    -c:v libx264 -preset ultrafast -tune zerolatency -crf 23 \
    -f rtsp -rtsp_transport tcp \
    rtsp://localhost:8554/zed_depth \
    > /tmp/nomad_video/ffmpeg_depth.log 2>&1 &
```

## Troubleshooting

### Stream shows [--] in Mission Planner

- Check if the video bridge is running: `docker exec nomad_isaac_ros pgrep -af ros_video_bridge`
- Check if FFmpeg encoder is running: `pgrep -af ffmpeg`
- Check bridge logs: `docker exec nomad_isaac_ros cat /tmp/video_bridge_rgb.log`
- Check encoder logs: `cat /tmp/nomad_video/ffmpeg_rgb.log`

### FFmpeg encoder fails

- Ensure the video bridge started first (it creates the TCP server)
- Check if the TCP port is already in use: `netstat -tulpn | grep 9999`
- Try restarting: `~/NOMAD/scripts/start_multi_video.sh restart`

### ROS topic doesn't exist

- Check available topics: `docker exec nomad_isaac_ros bash -c "source /opt/ros/humble/setup.bash && ros2 topic list | grep zed"`
- Ensure ZED camera is connected and Isaac ROS is running
- Check ZED launch parameters in `scripts/start_isaac_ros_auto.sh`

### No streams appear after refresh

- Check MediaMTX API: `curl http://localhost:9997/v3/paths/list`
- Ensure MediaMTX API authentication is disabled in `infra/mediamtx.yml`
- Restart MediaMTX if needed

## Performance Notes

- Each stream uses ~4-6 Mbps bandwidth (H.264 @ CRF 23)
- Running 3 streams simultaneously uses ~12-18 Mbps
- CPU usage: ~15-20% per FFmpeg encoder on Jetson Orin Nano
- Consider reducing FPS or resolution if performance is an issue

## Files Modified

- `edge_core/ros_video_bridge.py` - Added `--tcp-port` parameter
- `scripts/start_multi_video.sh` - New script for managing multiple streams
- `scripts/start_nomad_full.sh` - Updated to use multi-stream system
- `config/video_streams.json` - Stream configuration
- `mission_planner/src/EmbeddedVideoPlayer.cs` - Stream selector with live status

AEAC 2026 - McGill Aerial Design
