#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Check nvblox node status
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null

  echo "=== nvblox processes ==="
  ps aux | grep -i nvblox | grep -v grep || echo "NO_NVBLOX_PROCESS"

  echo ""
  echo "=== All ROS nodes ==="
  timeout 3 ros2 node list 2>/dev/null || echo "NODE_LIST_FAILED"

  echo ""
  echo "=== ZED nvblox launch log (last 40 lines) ==="
  tail -40 /tmp/zed_nvblox.log 2>/dev/null || echo "NO_ZED_NVBLOX_LOG"
'
