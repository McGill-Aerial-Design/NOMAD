#!/bin/bash
# Check mesh-related ROS2 topics and processes inside Isaac ROS container
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

  echo "=== ROS2 processes ==="
  ps aux | grep -E "ros2|zed|nvblox|bridge" | grep -v grep | head -20

  echo ""
  echo "=== Mesh-related topics ==="
  timeout 3 ros2 topic list 2>/dev/null | grep -iE "mesh|nvblox|marker|pointcloud|block" || echo "NO_MESH_TOPICS"

  echo ""
  echo "=== All ROS2 topics ==="
  timeout 3 ros2 topic list 2>/dev/null | head -40

  echo ""
  echo "=== Bridge processes ==="
  ps aux | grep -E "ros_http_bridge|simple_video" | grep -v grep
'
