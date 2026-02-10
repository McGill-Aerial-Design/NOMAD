# Jetson Video Streaming - Software-Encoded Solution

## Overview

The Jetson Orin Nano does **not** have a hardware video encoder (NVENC). All H.264 encoding is done in software using `openh264enc` via GStreamer, running inside the Isaac ROS Docker container.

**Key Facts:**
- Software H.264 encoding via openh264enc (GStreamer plugin)
- Runs inside the Isaac ROS container alongside the ZED ROS2 wrapper
- Managed by Edge Core's `VideoStreamManager` on the host
- Streams to MediaMTX RTSP server for multi-viewer distribution
- Dynamic topic switching via REST API (no stream URL change)

## Architecture

```
ROS2 Image Topic -> openh264enc (software) -> RTSP -> MediaMTX -> Mission Planner
```

## Build & Deploy

The video bridge runs inside the Isaac ROS container. No separate container build is needed.

### Start Streaming

```bash
# Edge Core auto-starts the video bridge when Isaac ROS is running
# Or manually via API:
curl -X POST http://100.85.121.98:8000/api/video/start
```

### View Stream

```bash
# From Windows - open in VLC:
vlc rtsp://100.85.121.98:8554/primary --rtsp-tcp --network-caching=200

# Or FFplay:
ffplay -fflags nobuffer -flags low_delay -rtsp_transport tcp rtsp://100.85.121.98:8554/primary
```

## Configuration

Set in `config/env/jetson.env`:

```bash
NOMAD_VIDEO_AUTO_START=true     # Auto-start stream on Edge Core startup
```

Default stream parameters (set in `video_stream_manager.py`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| Resolution | 1280x720 | Output video size |
| FPS | 30 | Target frame rate |
| Bitrate | 4 Mbps | H.264 encoding bitrate |
| Encoder | openh264enc | Software encoder (no HW encoder on Orin Nano) |

## API Endpoints

All on Edge Core (port 8000):

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/video/topics` | GET | List available ROS2 image topics |
| `/api/video/source?topic=...` | POST | Switch camera view (RTSP URL stays the same) |
| `/api/video/status` | GET | Current stream status (fps, frames, errors) |
| `/api/video/start` | POST | Start streaming |
| `/api/video/stop` | POST | Stop streaming |
| `/api/video/restart` | POST | Restart stream |
| `/api/video/logs?lines=50` | GET | View bridge logs |

## Troubleshooting

### No video stream

```bash
# 1. Check if ZED is publishing topics
ssh mad@100.85.121.98 "docker exec nomad_isaac_ros bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null; source /opt/ros/humble/install/setup.bash; ros2 topic list | grep zed'"

# 2. Check video bridge status
curl http://100.85.121.98:8000/api/video/status

# 3. Check MediaMTX
curl http://100.85.121.98:9997/v3/paths/list

# 4. View bridge logs
curl http://100.85.121.98:8000/api/video/logs
```

### Low FPS or choppy video

Software encoding on Orin Nano uses CPU. If CPU is under heavy load:
- Reduce resolution (try 640x480)
- Lower bitrate (try 2 Mbps)
- Reduce FPS (try 15)
- Check CPU load: `curl http://100.85.121.98:8000/health/detailed`

## Performance

- **CPU Usage:** ~15-25% (software encoding)
- **GPU Usage:** 0% (encoding is CPU-only)
- **Memory:** ~200MB (ROS2 + Python bridge)
- **Latency:** 100-200ms (camera to RTSP viewer)
- **Bitrate:** Configurable, default 4 Mbps

## Files

| File | Description |
|------|-------------|
| `edge_core/ros/simple_video_bridge.py` | ROS2 to RTSP bridge using openh264enc |
| `edge_core/video_stream_manager.py` | Host-side manager (starts/stops bridge in container) |
| `edge_core/api.py` | API endpoints (`/api/video/*`) |
| `infra/mediamtx.yml` | MediaMTX RTSP server config |
| `infra/docker/Dockerfile.jetson_video` | Standalone video container (alternative deployment) |

---
AEAC 2026 - McGill Aerial Design
