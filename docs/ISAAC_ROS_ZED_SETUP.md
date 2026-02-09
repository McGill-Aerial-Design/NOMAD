# Isaac ROS Nvblox with ZED Camera - Complete Setup Guide

This document contains all the extra steps needed beyond the official Isaac ROS documentation to get Nvblox working with a ZED camera.

## Understanding the Docker Workflow

**What persists between container restarts:**
- Source code in `${ISAAC_ROS_WS}/src/`
- Build artifacts in `${ISAAC_ROS_WS}/build/`, `install/`, `log/`

**What gets reset each time:**
- System packages installed with `apt install`
- ZED SDK installation
- Any container filesystem changes

This means you'll need to reinstall certain dependencies each time you restart the container.

## Initial Setup (One-Time)

Follow the official Isaac ROS documentation:
1. Complete the ZED Setup Tutorial (steps 1-7)
2. Complete the Nvblox Quickstart
3. Download the quickstart assets

**Important notes:**
- In step 6 of ZED setup, you rebuild the Docker image with ZED SDK
- After step 6, **exit and re-enter the container** before continuing
- Use `camera_model:=zed2i` for ZED 2i cameras (confirmed by NVIDIA)

## Every Time You Restart the Container

Each time you run `./src/isaac_ros_common/scripts/run_dev.sh`, you need to:

### 1. Install Required Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-zed-msgs \
  ros-humble-robot-localization \
  ros-humble-point-cloud-transport \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools
```

### 2. Source Your Workspace

```bash
cd /workspaces/isaac_ros-dev
source install/setup.bash
```

### 3. (Optional) Verify ZED Camera Connection

```bash
/usr/local/zed/tools/ZED_Explorer
```

Press Ctrl+C to exit after verification.

## Automated Setup Script

To save time, create this script inside the container:

```bash
cat > ~/setup_isaac_zed.sh << 'EOF'
#!/bin/bash
echo "Installing dependencies..."
sudo apt update
sudo apt install -y \
  ros-humble-zed-msgs \
  ros-humble-robot-localization \
  ros-humble-point-cloud-transport \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools

echo "Sourcing workspace..."
cd /workspaces/isaac_ros-dev
source install/setup.bash

echo "✓ Setup complete! Ready to launch."
EOF
chmod +x ~/setup_isaac_zed.sh
```

**Then each time you enter the container, just run:**

```bash
~/setup_isaac_zed.sh
```

## Running the ZED Example

After running the setup script:

```bash
ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2i
```

Replace `zed2i` with your camera model if different (`zed`, `zed2`, `zedx`, `zedm`, etc.)

## Common Issues and Solutions

### Issue: `ros2 pkg list` doesn't show zed_wrapper

**Solution:** Source your workspace
```bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

### Issue: Library loading errors (libpoint_cloud_transport.so, librobot_localization, etc.)

**Solution:** Install missing dependencies (see step 1 above)

### Issue: Transform lookup errors for `zed_camera_link`

**Solution:**
- Make sure ZED node is actually running (check for errors in launch output)
- Verify TF2 packages are installed
- Check camera is connected and detected

### Issue: Need to rebuild every time

**Solution:** You shouldn't need to rebuild if you didn't change code. The build artifacts in `install/` persist. Just:
1. Install apt dependencies
2. Source the workspace

## Key Concepts

**TF2 (Transform Library):** Tracks coordinate frame relationships (where the camera is relative to the robot/map)

**Point Cloud Transport:** Handles efficient transmission of 3D point cloud data from the ZED camera

**Robot Localization:** Provides sensor fusion and odometry tracking

**Nvblox:** NVIDIA's 3D reconstruction library that creates meshes and distance fields from sensor data

## Verification Commands

```bash
# Check if packages are available
ros2 pkg list | grep zed
ros2 pkg list | grep nvblox

# Check topics being published
ros2 topic list

# View transform tree
ros2 run tf2_tools view_frames

# Check ZED-specific topics
ros2 topic list | grep zed
```

## Quick Reference

**Container workspace:** `/workspaces/isaac_ros-dev`

**Install dependencies:** `~/setup_isaac_zed.sh` (after creating the script above)

**Launch command:**
```bash
ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2i
```

---

**Last Updated:** February 9, 2026  
**Isaac ROS Version:** 3.2  
**ROS Distribution:** Humble
