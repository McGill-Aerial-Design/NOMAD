#!/bin/bash
# Check mesh topic rate and bridge message receipt
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

  echo "=== Mesh topic rate (6s) ==="
  timeout 6 ros2 topic hz /nvblox_node/mesh --window 5 2>&1 | tail -3

  echo ""
  echo "=== Mesh message sample ==="
  timeout 5 ros2 topic echo /nvblox_node/mesh --once 2>&1 | head -20

  echo ""
  echo "=== Bridge stats log ==="
  grep -E "stats|mesh_received|vio_received" /tmp/ros_bridge2.log 2>/dev/null | tail -5
'
