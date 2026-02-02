# NOMAD Video Streaming System

Isaac ROS H264 encoder-based video streaming with dynamic topic switching.

## Architecture

```
ZED Camera -> Isaac ROS H264 Encoder -> CompressedImage (H264) -> RTSP Bridge -> MediaMTX -> Mission Planner
                                                    ^
                                                    |-- Topic switch via API (restarts encoder)
```

### Key Features

- **Single RTSP URL**: `rtsp://jetson:8554/primary` - never changes
- **Dynamic Topic Switching**: Change ZED camera view via API
- **Hardware Encoding**: Isaac ROS H264 encoder with NVIDIA hardware acceleration
- **Multiple Viewers**: MediaMTX distributes to any number of clients

### Components

1. **Isaac H264 RTSP Bridge** (`edge_core/ros/isaac_h264_rtsp_bridge.py`)
   - Runs inside Isaac ROS container
   - Launches Isaac ROS H264 encoder node for encoding
   - Subscribes to H264 CompressedImage output
   - Pushes raw H264 NAL units to MediaMTX via RTSP
   - HTTP control API on port 9200

2. **Video Stream Manager** (`edge_core/video_stream_manager.py`)
   - Runs on Jetson host
   - Controls bridge via HTTP
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

**Important**: The RTSP URL stays constant. The Isaac ROS H264 encoder restarts with the new topic.

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

Replace `jetson` with your Tailscale IP (e.g., `100.75.218.89`).

```bash
# VLC
vlc rtsp://100.75.218.89:8554/primary --rtsp-tcp --network-caching=200

# FFplay
ffplay -fflags nobuffer -flags low_delay -rtsp_transport tcp rtsp://100.75.218.89:8554/primary
```

## Mission Planner Integration

The video panel in Mission Planner includes:

1. **Topic Dropdown**: Select which ZED view to stream
2. **Refresh Button (...)**: Query Jetson for available topics
3. **Latency Slider**: Adjust network caching (50-1000ms)
4. **Play/Stop/Snapshot**: Standard video controls

When you select a topic from the dropdown:
- The API switches the source topic on the Jetson
- The Isaac ROS H264 encoder restarts with the new topic
- The RTSP URL stays the same
- The video player does NOT reconnect (GStreamer handles stream continuity)
- Brief interruption (~500ms) during encoder restart

### Configuration

### Environment Variables

Set in `config/env/jetson.env`:

```bash
NOMAD_VIDEO_AUTO_START=true     # Auto-start stream on Edge Core startup
```

### Isaac H264 RTSP Bridge Parameters

The bridge accepts these command-line arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `--source-topic` | `/zed/zed_node/rgb/image_rect_color` | Source ROS image topic |
| `--h264-topic` | `/image_compressed` | H264 CompressedImage topic |
| `--rtsp-url` | `rtsp://172.17.0.1:8554/primary` | MediaMTX RTSP URL |
| `--http-port` | `9200` | HTTP control API port |
| `--width` | `1280` | Output video width |
| `--height` | `720` | Output video height |

### Isaac ROS H264 Encoder Parameters

The encoder uses these settings (configured in bridge):

| Parameter | Value | Description |
|-----------|-------|-------------|
| `config` | `pframe_cqp` | P-frame constant QP preset |
| `qp` | `20` | Quality factor (lower = better quality) |
| `iframe_interval` | `30` | I-frame every 30 frames (~1 sec at 30fps) |

## Troubleshooting

### Stream not playing

1. Check if bridge is running:
```bash
ssh mad@100.75.218.89 "docker exec nomad_isaac_ros_32 pgrep -af isaac_h264_rtsp_bridge"
```

2. Check if encoder is running:
```bash
ssh mad@100.75.218.89 "docker exec nomad_isaac_ros_32 pgrep -af encoder_node"
```

3. Check bridge logs:
```bash
curl http://100.75.218.89:8000/api/video/logs
```

4. Check if MediaMTX is running:
```bash
ssh mad@100.75.218.89 "curl -s http://localhost:9997/v3/paths/list"
```

### No topics available

1. Check if ZED wrapper is running:
```bash
ssh mad@100.75.218.89 "docker exec nomad_isaac_ros_32 ros2 topic list | grep zed"
```

2. If no topics, restart Isaac ROS container:
```bash
ssh mad@100.75.218.89
cd ~/NOMAD
./scripts/start_isaac_ros_auto.sh restart
```

### High latency

1. Reduce network caching in VLC to 100-200ms
2. Check Jetson CPU/GPU load: `curl http://100.75.218.89:8000/health/detailed`
3. Try reducing resolution or bitrate via API

### Topic switch not working

1. Check API response for errors:
```bash
curl -X POST "http://100.75.218.89:8000/api/video/source?topic=/zed/zed_node/left/image_rect_color"
```

2. Check bridge HTTP is accessible:
```bash
ssh mad@100.75.218.89 "curl http://localhost:9200/status"
```

Note: Topic switching restarts the Isaac ROS H264 encoder, so there may be a brief interruption.

## Performance

- **Latency**: ~100-150ms glass-to-glass
- **Bandwidth**: ~4 Mbps (configurable via encoder QP)
- **CPU Usage**: ~10% on Jetson (encoder offloaded to NVENC)
- **GPU Usage**: ~25% (Isaac ROS H264 encoder)
- **Topic Switch Time**: ~500ms (encoder restart)

## Files

| File | Description |
|------|-------------|
| `edge_core/ros/isaac_h264_rtsp_bridge.py` | Bridge launching Isaac ROS encoder and pushing H264 to RTSP |
| `edge_core/video_stream_manager.py` | Host-side manager class |
| `edge_core/api.py` | API endpoints (`/api/video/*`) |
| `mission_planner/src/EmbeddedVideoPlayer.cs` | Video player UI |
| `infra/mediamtx.yml` | MediaMTX RTSP server config |

---
AEAC 2026 - McGill Aerial Design
