#!/bin/bash
# Check color_layer_marker contents
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

  echo "=== color_layer_marker rate (6s) ==="
  timeout 6 ros2 topic hz /nvblox_node/color_layer_marker --window 5 2>&1 | tail -3

  echo ""
  echo "=== color_layer_marker sample ==="
  timeout 8 ros2 topic echo /nvblox_node/color_layer_marker --once 2>&1 | head -30

  echo ""
  echo "=== Bridge mesh receive count (from log) ==="
  grep -E "Mesh sent|mesh_received|Voxel|voxel_marker|block" /tmp/ros_bridge2.log 2>/dev/null | tail -10
'
