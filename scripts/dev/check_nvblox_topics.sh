#!/bin/bash
# Check what nvblox is actually publishing
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

  echo "=== nvblox topics ==="
  timeout 3 ros2 topic list | grep nvblox | sort

  echo ""
  echo "=== color_layer_marker sample ==="
  timeout 5 ros2 topic echo /nvblox_node/color_layer_marker --once 2>/dev/null | head -12

  echo ""
  echo "=== Mesh sample ==="
  timeout 5 ros2 topic echo /nvblox_node/mesh --once 2>/dev/null | head -12

  echo ""
  echo "=== Bridge subscription log ==="
  docker logs nomad_isaac_ros 2>/dev/null | grep -i "subscribed\|mesh\|voxel" | tail -5 2>/dev/null || docker exec nomad_isaac_ros tail -20 /tmp/full_launch2.log 2>/dev/null | grep -iE "subscribed|mesh|voxel"
'
