#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Detailed container process and log analysis
docker exec nomad_isaac_ros bash -c '
  echo "=== ALL processes with timestamps ==="
  ps -eo pid,start,etimes,comm --sort=start_time | grep -vE "bash|ps |grep|cat " | tail -30

  echo ""
  echo "=== ZED node status ==="
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
  timeout 3 ros2 node list 2>/dev/null | grep zed || echo "NO_ZED_NODE"

  echo ""
  echo "=== VIO odom topic publishers ==="
  timeout 3 ros2 topic info /zed/zed_node/odom -v 2>/dev/null | head -15

  echo ""
  echo "=== Image topic publishers ==="
  timeout 3 ros2 topic info /zed/zed_node/rgb/image_rect_color -v 2>/dev/null | head -15

  echo ""
  echo "=== ZED topics actually publishing (one sample) ==="
  timeout 3 ros2 topic hz /zed/zed_node/odom --window 5 2>&1 | tail -3
'
