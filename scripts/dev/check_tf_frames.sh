#!/bin/bash
# Check TF frames and ZED camera pose
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

  echo "=== TF frames ==="
  timeout 3 ros2 run tf2_ros tf2_echo odom zed_camera_link 2>/dev/null | head -20

  echo ""
  echo "=== ZED base_link to camera ==="
  timeout 3 ros2 run tf2_ros tf2_echo zed_camera_link zed_left_camera_optical_frame 2>/dev/null | head -20

  echo ""
  echo "=== odom to base_link ==="
  timeout 3 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -20
'
