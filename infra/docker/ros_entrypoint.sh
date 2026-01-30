#!/bin/bash
# ==============================================================
# ROS2 Entrypoint Script for NOMAD Isaac ROS Container
# 
# This script:
# 1. Sources ROS2 Humble setup
# 2. Sources local workspace if built
# 3. Sets up environment variables
# 4. Executes the provided command
# ==============================================================
set -e

# Source ROS2 Humble setup
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[ros_entrypoint] Sourced ROS2 Humble"
fi

# Source local workspace if it exists (colcon build output)
if [ -f /workspaces/isaac_ros-dev/install/setup.bash ]; then
    source /workspaces/isaac_ros-dev/install/setup.bash
    echo "[ros_entrypoint] Sourced local workspace"
fi

# Source Isaac ROS workspace if it exists
if [ -f /opt/isaac_ros_ws/install/setup.bash ]; then
    source /opt/isaac_ros_ws/install/setup.bash
    echo "[ros_entrypoint] Sourced Isaac ROS workspace"
fi

# Print ROS2 environment info
echo "[ros_entrypoint] ROS_DISTRO: ${ROS_DISTRO:-not set}"
echo "[ros_entrypoint] ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo "[ros_entrypoint] RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-default}"

# Execute the provided command
exec "$@"
