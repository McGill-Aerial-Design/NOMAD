#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Full state check: ZED node + nvblox + mesh inside container
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

  echo "=== ROS nodes ==="
  timeout 3 ros2 node list 2>/dev/null

  echo ""
  echo "=== Topic publishers ==="
  for topic in /zed/zed_node/odom /zed/zed_node/left/image_rect_color /nvblox_node/mesh /nvblox_node/color_layer_marker; do
    count=$(timeout 2 ros2 topic info $topic 2>/dev/null | grep "Publisher count:" | awk "{print \$3}")
    echo "  $topic: ${count:-?} publishers"
  done

  echo ""
  echo "=== nvblox processes ==="
  ps aux | grep -iE "nvblox|component_container" | grep -v grep || echo "NONE"

  echo ""
  echo "=== Odom topic rate ==="
  timeout 4 ros2 topic hz /zed/zed_node/odom --window 10 2>&1 | tail -2
'
