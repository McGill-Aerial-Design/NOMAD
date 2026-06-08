#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Probe the actual nvblox Mesh message structure inside the Isaac ROS container.
# Run on the Jetson to diagnose mesh data flow issues.
# edge_core/ is mounted at /workspaces/isaac_ros-dev/edge_core/ inside the container.
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
  python3 /workspaces/isaac_ros-dev/edge_core/probe_mesh_msg.py
'
