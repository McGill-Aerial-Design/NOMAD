#!/bin/bash
# ==============================================================
# NOMAD nvblox Performance Launch Script
# 
# Launches nvblox with memory-optimized settings for Jetson Orin Nano
# Run inside the Isaac ROS container
# ==============================================================

set -e

# Configuration
CONFIG_FILE="/workspaces/isaac_ros-dev/config/nvblox_performance.yaml"
CAMERA_MODEL="zed2"  # zed2i uses zed2 profile

echo "=== NOMAD nvblox Performance Mode ==="
echo "Config: $CONFIG_FILE"
echo "Camera: $CAMERA_MODEL"
echo ""

# Check if config exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "ERROR: Config file not found: $CONFIG_FILE"
    echo "Make sure the config directory is mounted in Docker"
    exit 1
fi

# Source ROS2
source /opt/ros/humble/install/setup.bash
if [ -f /workspaces/isaac_ros-dev/install/setup.bash ]; then
    source /workspaces/isaac_ros-dev/install/setup.bash
fi

# Check memory before launch
echo "Memory before launch:"
free -h

# Launch nvblox with performance config
echo ""
echo "Launching ZED + nvblox (performance mode)..."
ros2 launch nvblox_examples_bringup zed_example.launch.py \
    camera:=$CAMERA_MODEL \
    nvblox_params_file:=$CONFIG_FILE
