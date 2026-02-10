# NOMAD nvblox Visualization Quick Reference

## Overview
This document describes how to run nvblox 3D mesh visualization with the ZED camera on the Jetson.

## Quick Start

### From Windows PowerShell:
```powershell
ssh mad@100.85.121.98 "bash /home/mad/NOMAD/scripts/start_nvblox_visualization.sh"
```

### Manual Steps (if script fails):

1. **Check ZED camera is connected**:
```bash
ssh mad@100.85.121.98 "lsusb | grep -i stereo"
# Should show: STEREOLABS ZED 2i
```

2. **Restart Docker container** (required for fresh USB device access):
```bash
ssh mad@100.85.121.98 "docker restart nomad_isaac_ros"
# Wait 15-20 seconds
```

3. **Enable X11 display access**:
```bash
ssh mad@100.85.121.98 "export DISPLAY=:1 && export XAUTHORITY=/run/user/1000/gdm/Xauthority && xhost +local:docker"
```

4. **Launch full ZED + nvblox stack**:
```bash
ssh mad@100.85.121.98 "docker exec -d nomad_isaac_ros /bin/bash -c 'source /workspaces/isaac_ros-dev/install/setup.bash && ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2 > /tmp/zed_nvblox_full.log 2>&1'"
# Wait 30 seconds for ZED calibration
```

5. **Launch RViz**:
```bash
ssh mad@100.85.121.98 "docker exec -e DISPLAY=:1 -e XAUTHORITY=/run/user/1000/gdm/Xauthority nomad_isaac_ros /bin/bash -c 'source /workspaces/isaac_ros-dev/install/setup.bash && rviz2 -d /workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/visualization/zed_example.rviz'"
```

## Key Topics

| Topic | Rate | Description |
|-------|------|-------------|
| `/nvblox_node/mesh` | ~7 Hz | 3D mesh visualization |
| `/nvblox_node/color_layer_marker` | ~7 Hz | Color layer markers |
| `/nvblox_node/static_map_slice` | ~1 Hz | 2D occupancy slice |
| `/zed/zed_node/left/image_rect_color` | 30 Hz | Left camera image |
| `/zed/zed_node/depth/depth_registered` | 30 Hz | Depth data |
| `/zed/zed_node/odom` | 30 Hz | Visual odometry |

## Troubleshooting

### ZED Camera Not Detected
- Check USB connection: `lsusb | grep -i stereo`
- Restart the Docker container: `docker restart nomad_isaac_ros`
- The ZED SDK sometimes needs a fresh USB bind after container restart

### nvblox Mesh Not Showing
- Check mesh topic is publishing: `ros2 topic hz /nvblox_node/mesh`
- In RViz, set Fixed Frame to `odom` or `map`
- Ensure NvbloxMesh display is enabled in RViz Displays panel
- Wait 10-20 seconds for mesh to build up

### RViz Not Launching
- Verify X11 setup: `export DISPLAY=:1`
- Run `xhost +local:docker` on the Jetson
- Check logs: `cat /tmp/rviz.log`

### TF Frame Errors
- The ZED driver publishes frames: `zed_camera_link`, `zed_left_camera_frame`, etc.
- nvblox expects proper TF chain from camera to base_link
- The zed_example.launch.py sets up correct TF automatically

## Configuration

### Docker Container
- Name: `nomad_isaac_ros`
- Image: `dustynv/ros:humble-ros-base-l4t-r36.2.0`
- Workspace: `/workspaces/isaac_ros-dev/`

### X11 Display
- DISPLAY: `:1`
- XAUTHORITY: `/run/user/1000/gdm/Xauthority`
- Requires `xhost +local:docker` for Docker GUI access

### RViz Config
- Path: `/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/visualization/zed_example.rviz`
- Contains: NvbloxMesh, PointCloud2, Image, TF displays

### ZED Camera
- Model: ZED 2i
- Launch parameter: `camera:=zed2`
- Resolution download may take ~7 minutes on first run

## Logs

| Log File | Contents |
|----------|----------|
| `/tmp/zed_nvblox_full.log` | Full ZED + nvblox launch output |
| `/tmp/rviz.log` | RViz startup log |
| `/tmp/zed_launch.log` | ZED camera only log |

## Stop Commands

```bash
# Stop all ROS nodes
ssh mad@100.85.121.98 "docker exec nomad_isaac_ros pkill -f 'ros2|rviz'"

# Stop just RViz
ssh mad@100.85.121.98 "docker exec nomad_isaac_ros pkill rviz2"
```
