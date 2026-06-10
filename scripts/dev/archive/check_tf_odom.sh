#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Check all available TF frames and ZED odom orientation
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

  echo "=== Available TF frames ==="
  timeout 3 ros2 topic echo /tf_static --once 2>/dev/null | grep frame_id | sort -u

  echo ""
  echo "=== odom to zed_camera_link ==="
  timeout 4 ros2 run tf2_ros tf2_echo odom zed_camera_link 2>/dev/null | head -12

  echo ""
  echo "=== Odom topic sample (raw VIO pose) ==="
  timeout 3 ros2 topic echo /zed/zed_node/odom --once 2>/dev/null | grep -A5 orientation
'
