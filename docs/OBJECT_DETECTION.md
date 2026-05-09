# YOLO26 Object Detection Integration

## Overview

NOMAD uses a custom YOLO26 model trained to detect colored circles (competition targets). Detection runs on the Jetson Orin Nano GPU via the ZED SDK's built-in custom object detection pipeline, which provides automatic TensorRT optimization and 3D localization via stereo depth.

## Architecture

```
ZED Camera (stereo)
    |
    v
ZED SDK (inside Isaac ROS container)
    |-- Stereo depth estimation
    |-- Custom ONNX model inference (TensorRT auto-optimized)
    |-- 3D object localization from depth + 2D detection
    |
    v
ROS2 Topic: /zed/zed_node/obj_det/objects (zed_interfaces/ObjectsStamped)
    |
    v
ros_http_bridge.py (subscribes, forwards via HTTP)
    |
    v
Edge Core API: POST /api/detections/update
    |-- Stores current frame detections
    |-- Maintains persistent detection history (deduplicated by proximity)
    |-- Streams to Mission Planner via /ws/slam WebSocket
    |-- Included in Task 1 capture metadata for AI description
    |
    v
Mission Planner SLAM3DView (renders colored 3D spheres at detection positions)
```

## Detection Classes

The YOLO26 model detects 6 classes of colored circles:

| Class ID | Label | Color in 3D View |
|----------|-------|-------------------|
| 0 | black_circle | Dark gray |
| 1 | blue_circle | Blue |
| 2 | green_circle | Green |
| 3 | red_circle | Red |
| 4 | white_circle | White |
| 5 | yellow_circle | Yellow |

## Setup

### 1. Convert Model to ONNX

The YOLO26 `.pt` model must be converted to ONNX format for the ZED SDK:

```bash
# On development machine (Windows or Linux with Python)
pip install ultralytics
python scripts/build/convert_yolo_to_onnx.py --input yolo26.pt --output config/models/best.onnx --imgsz 512
```

### 2. Deploy to Jetson

```bash
scp config/models/best.onnx mad@100.85.121.98:~/NOMAD/config/models/best.onnx
```

### 3. First Launch

On first launch with the ONNX model, the ZED SDK will auto-convert it to a TensorRT engine. This takes several minutes on Jetson Orin Nano but only happens once. The optimized engine is cached at `/usr/local/zed/resources/`.

### 4. Launch with Detection Enabled

Detection is enabled by default in the updated launch file:

```bash
# With detection (default)
ros2 launch /workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py

# Without detection
ros2 launch /workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py enable_od:=false
```

## Configuration

### ZED Custom OD Config

File: `config/custom_circle_detection.yaml`

Key parameters:
- `custom_onnx_file`: Path to ONNX model inside container
- `custom_onnx_input_size`: 512 (must match export resolution)
- `custom_class_count`: 6
- Per-class `confidence_threshold`: 40% (adjustable)
- `is_static: true` -- targets don't move
- `tracking_timeout`: 5 seconds

### ROS-HTTP Bridge

The bridge auto-subscribes to `/zed/zed_node/obj_det/objects` and forwards detections at 5Hz:

```bash
python3 ros_http_bridge.py --host 172.17.0.1 --port 8000 --detection-topic /zed/zed_node/obj_det/objects
```

Disable with `--disable-detections`.

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/detections/update` | Receive detections from ROS bridge |
| GET | `/api/detections` | Get current + history detections |
| GET | `/api/detections/status` | Get detection pipeline status |
| GET | `/api/detections/summary` | Get detections grouped by class |
| POST | `/api/detections/start` | Start detection pipeline |
| POST | `/api/detections/stop` | Stop detection pipeline |
| DELETE | `/api/detections/history` | Clear detection history |

### Example: Get Detection Summary

```json
GET /api/detections/summary
{
  "total_unique_targets": 8,
  "by_class": {
    "red_circle": {
      "count": 3,
      "avg_confidence": 0.85,
      "positions": [
        {"x": 1.2, "y": 0.5, "z": -0.3},
        {"x": 2.1, "y": -1.0, "z": -0.2}
      ]
    },
    "blue_circle": {
      "count": 2,
      "avg_confidence": 0.78,
      "positions": [...]
    }
  }
}
```

## 3D Visualization

Detections appear as colored spheres in the Mission Planner SLAM3DView:

- Sphere color matches the detected circle color
- Sphere size scales with detection confidence
- A thin vertical line connects each marker to the ground plane for depth perception
- Markers are streamed via the `/ws/slam` WebSocket at ~5Hz
- Markers persist in the view based on detection history (deduplicated by proximity)

## Task 1 AI Integration

When a Task 1 photo is captured, the current detection history is included in the metadata JSON as `detected_targets`. The AI description prompt uses this data to:

- Confirm or correct automated detection results
- Describe visible colored circles/targets
- Count each color of target and approximate positions
- Identify new vs previously seen targets

## Files Modified/Created

| File | Change |
|------|--------|
| `scripts/build/convert_yolo_to_onnx.py` | New: YOLO .pt to ONNX conversion |
| `config/custom_circle_detection.yaml` | Updated: container model path |
| `config/launch/nomad_zed_nvblox.launch.py` | Updated: enable OD + custom config |
| `edge_core/ros_http_bridge.py` | Added: detection subscription + forwarding |
| `edge_core/api.py` | Added: detection endpoints + SLAM WS integration |
| `mission_planner/src/SLAM3DView.cs` | Added: 3D detection markers |
| `scripts/task1/process_task1_ai.py` | Updated: detection-aware AI prompt |
