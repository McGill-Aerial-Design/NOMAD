# NOMAD Video Streaming System

Software-encoded video streaming with dynamic topic switching via REST API.

## Architecture

```
ZED Camera -> ROS2 Image Topic -> openh264enc (software) -> RTSP -> MediaMTX -> Mission Planner
                                          ^
                                          |-- Topic switch via API (restarts bridge)
```

**Note:** The Jetson Orin Nano has no hardware video encoder (NVENC). All encoding
uses openh264enc (software) inside the Isaac ROS container.

### Key Features

- **Single RTSP URL**: `rtsp://jetson:8554/primary` -- never changes
- **Dynamic Topic Switching**: Change ZED camera view via API
- **Software Encoding**: openh264enc via GStreamer (no HW encoder on Orin Nano)
- **Multiple Viewers**: MediaMTX distributes to any number of clients

### Components

1. **Simple Video Bridge** (`edge_core/ros/simple_video_bridge.py`)
   - Runs inside Isaac ROS container
   - Subscribes to a ROS2 image topic
   - Encodes with openh264enc and pushes to MediaMTX via RTSP
   - HTTP control API on port 9200

2. **Video Stream Manager** (`edge_core/video_stream_manager.py`)
   - Runs on Jetson host (inside Edge Core)
   - Controls bridge via Docker exec and HTTP
   - Provides Edge Core API integration

3. **MediaMTX** (`infra/mediamtx.yml`)
   - RTSP server on port 8554
   - Distributes stream to multiple clients

## API Endpoints

All endpoints are on the Edge Core API (port 8000).

### List Available Topics

```bash
GET /api/video/topics
```

Response:
```json
{
  "topics": [
    {"name": "/zed/zed_node/rgb/image_rect_color", "display_name": "zed: rgb/image_rect_color"},
    {"name": "/zed/zed_node/left/image_rect_color", "display_name": "zed: left/image_rect_color"},
    {"name": "/zed/zed_node/right/image_rect_color", "display_name": "zed: right/image_rect_color"},
    {"name": "/zed/zed_node/depth/depth_registered", "display_name": "zed: depth/depth_registered"}
  ],
  "count": 4
}
```

### Switch Video Source

```bash
POST /api/video/source?topic=/zed/zed_node/left/image_rect_color
```

Response:
```json
{
  "success": true,
  "topic": "/zed/zed_node/left/image_rect_color",
  "rtsp_url": "rtsp://localhost:8554/primary"
}
```

The RTSP URL stays constant. The bridge restarts with the new topic.

### Get Current Status

```bash
GET /api/video/status
```

Response:
```json
{
  "streaming": true,
  "current_topic": "/zed/zed_node/rgb/image_rect_color",
  "rtsp_url": "rtsp://localhost:8554/primary",
  "fps": 29.8,
  "frame_count": 12345,
  "error_count": 0,
  "width": 1280,
  "height": 720,
  "bitrate_mbps": 4
}
```

### Start/Stop/Restart Stream

```bash
POST /api/video/start
POST /api/video/stop
POST /api/video/restart
```

### Get Logs

```bash
GET /api/video/logs?lines=50
```

## Usage

### Automatic Startup

The video stream starts automatically when:
1. Edge Core starts with `NOMAD_VIDEO_AUTO_START=true` (default)
2. Isaac ROS container is running
3. ZED camera is publishing topics

### Manual Control

```bash
# Start stream
curl -X POST http://jetson:8000/api/video/start

# List topics
curl http://jetson:8000/api/video/topics

# Switch to left camera
curl -X POST "http://jetson:8000/api/video/source?topic=/zed/zed_node/left/image_rect_color"

# Check status
curl http://jetson:8000/api/video/status

# Stop stream
curl -X POST http://jetson:8000/api/video/stop
```

### View Stream

The RTSP URL is always: `rtsp://jetson:8554/primary`

Replace `jetson` with the Tailscale IP (e.g., `100.85.121.98`).

```bash
# VLC
vlc rtsp://100.85.121.98:8554/primary --rtsp-tcp --network-caching=200

# FFplay
ffplay -fflags nobuffer -flags low_delay -rtsp_transport tcp rtsp://100.85.121.98:8554/primary
```

## Mission Planner Integration

The video panel in Mission Planner includes:

1. **Topic Dropdown**: Select which ZED view to stream
2. **Refresh Button (...)**: Query Jetson for available topics
3. **Latency Slider**: Adjust network caching (50-1000ms)
4. **Play/Stop/Snapshot**: Standard video controls

When you select a topic from the dropdown:
- The API switches the source topic on the Jetson
- The bridge restarts with the new topic
- The RTSP URL stays the same
- Brief interruption (~500ms) during bridge restart

## Troubleshooting

### Stream not playing

1. Check video bridge status:
```bash
curl http://100.85.121.98:8000/api/video/status
```

2. Check if ZED is publishing:
```bash
ssh mad@100.85.121.98 "docker exec nomad_isaac_ros bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null; source /opt/ros/humble/install/setup.bash; ros2 topic list | grep zed'"
```

3. Check bridge logs:
```bash
curl http://100.85.121.98:8000/api/video/logs
```

4. Check MediaMTX:
```bash
ssh mad@100.85.121.98 "curl -s http://localhost:9997/v3/paths/list"
```

### No topics available

1. Check if ZED wrapper is running:
```bash
ssh mad@100.85.121.98 "docker exec nomad_isaac_ros bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null; source /opt/ros/humble/install/setup.bash; ros2 topic list | grep zed'"
```

2. If no topics, restart Isaac ROS container:
```bash
ssh mad@100.85.121.98 "cd ~/NOMAD && ./scripts/run/start_isaac_ros_auto.sh restart"
```

### High latency or choppy video

Software encoding on Orin Nano uses CPU. Mitigation:
1. Reduce network caching in VLC to 100-200ms
2. Lower resolution or bitrate via API
3. Check Jetson CPU load: `curl http://100.85.121.98:8000/health/detailed`

## Performance

- **Latency**: ~100-200ms glass-to-glass
- **Bandwidth**: ~4 Mbps (configurable)
- **CPU Usage**: ~15-25% on Jetson (software encoding)
- **GPU Usage**: 0% (no hardware encoder on Orin Nano)
- **Topic Switch Time**: ~500ms (bridge restart)

## Files

| File | Description |
|------|-------------|
| `edge_core/ros/simple_video_bridge.py` | ROS2 to RTSP bridge using openh264enc |
| `edge_core/video_stream_manager.py` | Host-side manager class |
| `edge_core/api.py` | API endpoints (`/api/video/*`) |
| `mission_planner/src/EmbeddedVideoPlayer.cs` | Video player UI |
| `infra/mediamtx.yml` | MediaMTX RTSP server config |

---
AEAC 2026 - McGill Aerial Design
