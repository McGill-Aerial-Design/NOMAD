#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# ==============================================================
# ROS2 Entrypoint Script for NOMAD Isaac Sim Container (x86_64)
#
# This script:
# 1. Sources ROS2 Humble setup
# 2. Sources ZED ROS2 workspace
# 3. Sources Isaac ROS nvblox utils
# 4. Sources NOMAD workspace if mounted
# 5. Sets up environment variables
# 6. Executes the provided command
# ==============================================================
set -e

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[ros_entrypoint_sim] Sourced ROS2 Humble"
fi

if [ -f /opt/zed_ros_ws/install/setup.bash ]; then
    source /opt/zed_ros_ws/install/setup.bash
    echo "[ros_entrypoint_sim] Sourced ZED ROS2 workspace"
fi

if [ -f /opt/isaac_ros_ws/install/setup.bash ]; then
    source /opt/isaac_ros_ws/install/setup.bash
    echo "[ros_entrypoint_sim] Sourced Isaac ROS workspace"
fi

if [ -f /opt/isaac_ros_installed/setup.bash ]; then
    source /opt/isaac_ros_installed/setup.bash
    echo "[ros_entrypoint_sim] Sourced Isaac ROS nvblox utils"
fi

if [ -f /workspaces/nomad-sim/install/setup.bash ]; then
    source /workspaces/nomad-sim/install/setup.bash
    echo "[ros_entrypoint_sim] Sourced NOMAD workspace"
fi

echo "[ros_entrypoint_sim] ROS_DISTRO: ${ROS_DISTRO:-not set}"
echo "[ros_entrypoint_sim] ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo "[ros_entrypoint_sim] RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-default}"
echo "[ros_entrypoint_sim] NOMAD_SIM_MODE: ${NOMAD_SIM_MODE:-false}"
echo "[ros_entrypoint_sim] ISAAC_SIM_HEADLESS: ${ISAAC_SIM_HEADLESS:-1}"

exec "$@"
