# Edge Core

Onboard services for the Jetson Orin Nano (Python 3.13).

## Core Modules

| Module | Purpose |
|--------|---------|
| `main.py` | Entry point, service orchestration, runs FastAPI server |
| `api.py` | REST / WebSocket API endpoints (~125 endpoints across all tags) |
| `state.py` | Thread-safe state manager singleton |
| `models.py` | Pydantic models (DetectionInfo, etc.) |
| `mavlink_interface.py` | MAVLink telemetry + command interface with flight controller |
| `nav_controller.py` | Velocity / position command routing to ArduPilot |
| `servo_controller.py` | Cube Orange servo / relay commands via MAVLink |
| `spray_controller.py` | Fire-extinguisher spray control (Task 2) |
| `operational_mode.py` | Operational mode state machine |
| `video_stream_manager.py` | Video bridge / overlay / source switching |
| `isaac_ros_bridge.py` | Isaac ROS / nvblox container lifecycle |
| `ros_http_bridge.py` | ROS <-> HTTP bridge (runs inside Docker container) |
| `health_monitor.py` | Jetson hardware monitoring (CPU, GPU, temp, memory) |
| `time_manager.py` | NTP/GPS time synchronization |
| `geospatial.py` | GPS / ENU / NED coordinate conversions |
| `gdrive_upload.py` | Google Drive upload helpers |
| `ipc.py` | ZeroMQ inter-process communication for high-rate ROS data |
| `logging_service.py` | Structured mission event logging |
| `task2_circle_verify.py` | Task 2 circle detection verification |
| `target_localizer/` | HSV circle detection, building model, 3D back-projection, description generation |
| `ros/` | ROS2 utility nodes (VIO bridge, video bridge, obstacle distance, nav2 goal, etc.) |

## Quick Start

```bash
# Install dependencies (in venv)
pip install -r requirements.txt

# Run the server
python -m edge_core.main --port 8000
```

## API Endpoints

The full API is auto-documented at `/docs` (Swagger UI). Major endpoint groups:

| Group | Prefix | Key Endpoints |
|-------|--------|---------------|
| System | `/`, `/health`, `/status` | Service info, health checks, system state |
| Network | `/network/*` | Tailscale status, reconnect, ping |
| Task 1 | `/api/task/1/*` | Target capture/save/clear, building corners, model |
| Task 2 | `/api/task/2/*` | Exclusion map, target hits |
| VIO | `/api/vio/*` | Pose, trajectory, calibration, area save/load |
| Navigation | `/api/nav/*` | Velocity/position commands, stop, guided mode |
| Nav2 | `/api/nav2/*` | Autonomous path goals, feedback, results |
| Servo | `/api/servo/*` | Cube servo PWM, camera tilt, shooter relay, config |
| Spray | `/api/spray/*` | Spray trigger, abort, status |
| Isaac ROS | `/api/isaac/*` | Container start/stop, nvblox, logs |
| Detections | `/api/detections/*` | Start/stop pipeline, history, summary |
| Video | `/api/video/*` | Source switching, start/stop, overlay, bridges |
| Terminal | `/api/terminal/*` | Whitelisted commands, logs |
| WebSockets | `/ws/state`, `/ws/slam` | Real-time state (10Hz), SLAM mesh stream |
