#!/bin/bash
# Check mesh data flow in detail
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

  echo "=== Mesh topic info ==="
  timeout 3 ros2 topic info /nvblox_node/mesh -v 2>/dev/null || echo "TOPIC_INFO_FAILED"

  echo ""
  echo "=== Color layer marker topic info ==="  
  timeout 3 ros2 topic info /nvblox_node/color_layer_marker -v 2>/dev/null || echo "MARKER_INFO_FAILED"

  echo ""
  echo "=== Mesh topic rate (5s sample) ==="
  timeout 5 ros2 topic hz /nvblox_node/mesh --window 10 2>&1 | tail -3 || echo "NO_MESH_RATE"

  echo ""
  echo "=== Bridge log (last 30 lines) ==="
  tail -30 /tmp/ros_bridge.log 2>/dev/null || echo "NO_BRIDGE_LOG"
'
