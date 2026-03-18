#!/bin/bash
# Investigate and fix ZED depth publishing
set -e

JETSON="mad@100.85.121.98"

echo "=== Step 1: Check ZED ROS2 parameters ==="
ssh -o ConnectTimeout=10 "$JETSON" "docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash 2>/dev/null
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
  
  echo \"ZED node parameters:\"
  timeout 3 ros2 param list /zed_node 2>/dev/null | head -30 || echo \"Could not list params (DDS issue)\"
  
  echo \"\"
  echo \"Checking common depth params:\"
  timeout 3 ros2 param get /zed_node publish_depth 2>/dev/null || echo \"publish_depth not found\"
  timeout 3 ros2 param get /zed_node depth_topic 2>/dev/null || echo \"depth_topic not found\"
  timeout 3 ros2 param get /zed_node depth_qos 2>/dev/null || echo \"depth_qos not found\"
'"

echo ""
echo "=== Step 2: Check ZED wrapper config files ==="
ssh -o ConnectTimeout=10 "$JETSON" "docker exec nomad_isaac_ros bash -c '
  find /workspaces/isaac_ros-dev/install/zed_wrapper -name \"*.yaml\" 2>/dev/null | head -5
  echo \"\"
  echo \"Common YAML config:\"
  find /workspaces/isaac_ros-dev/install/zed_wrapper -name \"common.yaml\" 2>/dev/null && cat /workspaces/isaac_ros-dev/install/zed_wrapper/share/zed_wrapper/config/common.yaml | grep -i depth | head -10 || echo \"not found\"
'"

echo ""
echo "=== Step 3: Check ZED SDK depth availability ==="
ssh -o ConnectTimeout=10 "$JETSON" "docker exec nomad_isaac_ros bash -c '
  # Check if ZED camera supports depth (it should)
  lsusb | grep ZED || echo \"ZED not found in USB devices\"
  echo \"\"
  echo \"ZED node info (may fail due to DDS):\"
  timeout 2 ros2 node info /zed_node 2>/dev/null | grep -i depth || echo \"Could not get node info\"
'"
