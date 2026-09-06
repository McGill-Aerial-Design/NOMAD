#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# ==============================================================
# ROS2 entrypoint for the NOMAD Jetson (Isaac ROS) container.
#
# Sources ROS2 Humble plus any built workspace, prints the resolved ROS
# environment for debuggability, then exec's whatever command was passed
# (defaults to an interactive shell). The C++ adapter node is launched on top
# of this, e.g. `ros2 run nomad_ros nomad_vehicle_node --ros-args
# -p endpoint:=udpin:0.0.0.0:14552`.
# ==============================================================
set -e

# Source ROS2 Humble (the official Isaac ROS dev base provides this).
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    echo "[ros_entrypoint] sourced ROS2 Humble"
fi

# Source a colcon-built workspace if one is mounted/built.
if [ -f /workspaces/isaac_ros-dev/install/setup.bash ]; then
    # shellcheck disable=SC1091
    source /workspaces/isaac_ros-dev/install/setup.bash
    echo "[ros_entrypoint] sourced local workspace"
fi

# Source the sim-ros image's colcon workspace (nomad_ros + nomad_core).
if [ -f /ws/install/setup.bash ]; then
    # shellcheck disable=SC1091
    source /ws/install/setup.bash
    echo "[ros_entrypoint] sourced NOMAD workspace"
fi

# Source a prebuilt Isaac ROS workspace if present.
if [ -f /opt/isaac_ros_ws/install/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/isaac_ros_ws/install/setup.bash
    echo "[ros_entrypoint] sourced Isaac ROS workspace"
fi

echo "[ros_entrypoint] ROS_DISTRO=${ROS_DISTRO:-unset} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW=${RMW_IMPLEMENTATION:-default}"

exec "$@"
