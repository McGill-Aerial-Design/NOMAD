#!/bin/bash
# Probe the actual nvblox Mesh message structure inside the Isaac ROS container.
# Run on the Jetson to diagnose mesh data flow issues.
docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
  python3 /workspaces/isaac_ros-dev/scripts/dev/probe_mesh_msg.py
'
