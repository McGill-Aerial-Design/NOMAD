#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# nomad-isaac-sim service (local x86_64 simulation)
#
# Owns: the `nomad_isaac_sim` Docker container (Isaac Sim + ZED ROS2).
# This is the simulation counterpart to the Jetson's
# `nomad_isaac_ros_container` service. It provides an identical ROS2
# topic interface so Edge Core and Mission Planner cannot distinguish
# sim from real hardware.
#
# Depends on: Docker with NVIDIA Container Toolkit.
# Does NOT own the ros_http_bridge, video bridge, or nvblox — each of
# those runs inside this container via docker exec, same as on the Jetson.
# =============================================================================
set -u
SERVICE="isaac-sim"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
. "$SCRIPT_DIR/../lib/common.sh"

ISAAC_SIM_CONTAINER_NAME="${ISAAC_SIM_CONTAINER_NAME:-nomad_isaac_sim}"
ISAAC_SIM_IMAGE_NAME="${ISAAC_SIM_IMAGE_NAME:-nomad-isaac-sim:latest}"

container_running() {
    docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$ISAAC_SIM_CONTAINER_NAME"
}

create_container() {
    local image="$1"
    log_info "creating Isaac Sim container $ISAAC_SIM_CONTAINER_NAME from $image"
    docker run -d \
        --name "$ISAAC_SIM_CONTAINER_NAME" \
        --init \
        --runtime nvidia \
        --gpus all \
        --privileged \
        --network host \
        --ipc host \
        --shm-size 4g \
        -v "$NOMAD_REPO_ROOT/edge_core:/workspaces/nomad-sim/edge_core:ro" \
        -v "$NOMAD_REPO_ROOT/config:/workspaces/nomad-sim/config:ro" \
        -v "$NOMAD_REPO_ROOT/infra/mediamtx.yml:/workspaces/nomad-sim/infra/mediamtx.yml:ro" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e __NV_PRIME_RENDER_OFFLOAD=1 \
        -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
        -e NOMAD_SIM_MODE=true \
        -e NOMAD_ENABLE_VISION=true \
        -e ISAAC_SIM_HEADLESS="${ISAAC_SIM_HEADLESS:-1}" \
        -e ISAAC_SIM_WORLD="${ISAAC_SIM_WORLD:-}" \
        -e ISAAC_SIM_DRONE_START_X="${ISAAC_SIM_DRONE_START_X:-0.0}" \
        -e ISAAC_SIM_DRONE_START_Y="${ISAAC_SIM_DRONE_START_Y:-0.0}" \
        -e ISAAC_SIM_DRONE_START_Z="${ISAAC_SIM_DRONE_START_Z:-1.0}" \
        -e ISAAC_SIM_DRONE_START_YAW="${ISAAC_SIM_DRONE_START_YAW:-0.0}" \
        -e ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID:-0}" \
        -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        -e NOMAD_API_PORT="${NOMAD_API_PORT:-8000}" \
        -e NOMAD_API_KEY="${NOMAD_API_KEY:-}" \
        -e NOMAD_INTERNAL_TOKEN="${NOMAD_INTERNAL_TOKEN:-}" \
        -e ROS_HTTP_BRIDGE_RATE="${ROS_HTTP_BRIDGE_RATE:-5}" \
        -e ROS_HTTP_BRIDGE_VIO_TOPIC="${ROS_HTTP_BRIDGE_VIO_TOPIC:-/zed/zed_node/odom}" \
        -e ROS_HTTP_BRIDGE_MAG_TOPIC="${ROS_HTTP_BRIDGE_MAG_TOPIC:-/zed/zed_node/imu/mag}" \
        -e ROS_HTTP_BRIDGE_TRANSPORT="${ROS_HTTP_BRIDGE_TRANSPORT:-http}" \
        -e ZED_CAMERA_MODEL="${ZED_CAMERA_MODEL:-zed2i}" \
        -e ZED_CAMERA_NAME="${ZED_CAMERA_NAME:-zed}" \
        -e ZED_GRAB_RESOLUTION="${ZED_GRAB_RESOLUTION:-HD720}" \
        -e ZED_DEPTH_MODE="${ZED_DEPTH_MODE:-NEURAL_LIGHT}" \
        -e VIDEO_BRIDGE_WIDTH="${VIDEO_BRIDGE_WIDTH:-640}" \
        -e VIDEO_BRIDGE_HEIGHT="${VIDEO_BRIDGE_HEIGHT:-360}" \
        -e VIDEO_BRIDGE_FPS="${VIDEO_BRIDGE_FPS:-15}" \
        -e VIDEO_BRIDGE_BITRATE="${VIDEO_BRIDGE_BITRATE:-800}" \
        -e NOMAD_DEFAULT_VIDEO_TOPIC="${NOMAD_DEFAULT_VIDEO_TOPIC:-/zed/zed_node/rgb/color/rect/image}" \
        -w /workspaces/nomad-sim \
        "$image" \
        sleep infinity > /dev/null
}

svc_start() {
    if ! command -v docker >/dev/null 2>&1; then
        log_fail "docker is not installed"
        return 1
    fi

    if ! docker image inspect "$ISAAC_SIM_IMAGE_NAME" >/dev/null 2>&1; then
        log_fail "Isaac Sim image '$ISAAC_SIM_IMAGE_NAME' not found. Build it first:"
        log_fail "  docker build -f docker/Dockerfile.isaac_sim -t $ISAAC_SIM_IMAGE_NAME ."
        return 1
    fi

    if container_running; then
        log_ok "already running"
        return 0
    fi

    if docker ps -a --format '{{.Names}}' | grep -qx "$ISAAC_SIM_CONTAINER_NAME"; then
        log_info "container exists (stopped) — starting"
        docker start "$ISAAC_SIM_CONTAINER_NAME" >/dev/null
    else
        create_container "$ISAAC_SIM_IMAGE_NAME"
    fi

    if wait_for "docker exec $ISAAC_SIM_CONTAINER_NAME true" 15; then
        log_ok "container ready"
        return 0
    fi
    log_fail "container did not become ready in 15s"
    return 1
}

svc_stop() {
    if ! docker ps -a --format '{{.Names}}' | grep -qx "$ISAAC_SIM_CONTAINER_NAME"; then
        log_ok "container does not exist"
        return 0
    fi
    log_info "stopping container"
    docker stop "$ISAAC_SIM_CONTAINER_NAME" >/dev/null 2>&1 || true
    log_ok "stopped"
}

svc_status() {
    if container_running; then
        log_ok "running"
        return 0
    fi
    if docker ps -a --format '{{.Names}}' | grep -qx "$ISAAC_SIM_CONTAINER_NAME"; then
        log_warn "exists but stopped"
        return 1
    fi
    log_warn "does not exist (build image first)"
    return 1
}

svc_logs() {
    if ! docker ps -a --format '{{.Names}}' | grep -qx "$ISAAC_SIM_CONTAINER_NAME"; then
        log_warn "container does not exist"
        return 1
    fi
    docker logs --tail 100 -f "$ISAAC_SIM_CONTAINER_NAME"
}

dispatch_service "$@"
