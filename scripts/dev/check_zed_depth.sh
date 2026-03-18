#!/bin/bash
# Check ZED topics and data
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

  echo "=== ZED depth/color topics ==="
  timeout 2 ros2 topic list 2>/dev/null | grep -E "zed.*depth|zed.*rgb|zed.*image" | head -10

  echo ""
  echo "=== nvblox back_projected depth ==="
  timeout 3 ros2 topic info /nvblox_node/back_projected_depth/zed_left_camera_optical_frame 2>/dev/null | grep -E "Publisher|Subscription"
'
