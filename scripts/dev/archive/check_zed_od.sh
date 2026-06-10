#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Quick check: is zed_interfaces available in the Isaac ROS container?
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
  echo "=== ZED packages ==="
  ros2 pkg list 2>/dev/null | grep -i zed || echo "NO_ZED_PACKAGES"
  echo "=== ZED OD import test ==="
  python3 -c "
try:
    from zed_interfaces.msg import ObjectsStamped
    print(\"ZED_OD_AVAILABLE\")
except ImportError as e:
    print(\"ZED_OD_NOT_AVAILABLE:\", e)
" 2>&1
'
