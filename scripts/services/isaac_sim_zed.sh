#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# nomad-isaac-sim-zed service (local x86_64 simulation)
#
# Owns: the ZED simulation publisher inside the Isaac Sim container.
# This is the simulation counterpart to the Jetson's `zed_wrapper` service.
# Instead of launching a real ZED camera, it launches the ZED simulation
# publisher node which publishes the same ROS2 topics from Isaac Sim.
#
# Depends on: isaac_sim container running.
# =============================================================================
set -u
SERVICE="isaac-sim-zed"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
. "$SCRIPT_DIR/../lib/common.sh"

ISAAC_SIM_CONTAINER_NAME="${ISAAC_SIM_CONTAINER_NAME:-nomad_isaac_sim}"
PATTERN='zed_sim_publisher|isaac_sim_zed_bridge'
LAUNCH_SCRIPT_PATH=/tmp/nomad_isaac_sim_zed_launch.sh
LAUNCH_LOG=/tmp/nomad_isaac_sim_zed.log

sim_container_running() {
    docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$ISAAC_SIM_CONTAINER_NAME"
}

require_sim_container() {
    if ! sim_container_running; then
        log_fail "Isaac Sim container '$ISAAC_SIM_CONTAINER_NAME' is not running."
        log_fail "Start it with: nomad start isaac_sim  OR  docker compose -f docker/docker-compose.sim.yml up isaac_sim"
        return 1
    fi
}

write_launch_script() {
    local tmp
    tmp=$(mktemp)
    {
        echo "#!/bin/bash"
        ros_setup_prelude
        cat <<'EOS'
export PYTHONPATH=/opt/nomad/isaac_sim:/workspaces/nomad-sim/edge_core:/opt/nomad:${PYTHONPATH:-}

# Launch ZED simulation publisher (same topic interface as real ZED wrapper)
python3 /opt/nomad/isaac_sim/isaac_sim_zed_bridge.py \
    --headless \
    ${ISAAC_SIM_WORLD:+--world "$ISAAC_SIM_WORLD"} &

ZED_SIM_PID=$!

# Also launch the ZED sim publisher node for ROS2-only path
python3 /opt/nomad/isaac_sim/zed_sim_publisher.py &

PUBLISHER_PID=$!

# Wait for either to exit
wait $ZED_SIM_PID $PUBLISHER_PID 2>/dev/null
EOS
    } > "$tmp"
    docker cp "$tmp" "$ISAAC_SIM_CONTAINER_NAME:$LAUNCH_SCRIPT_PATH" >/dev/null
    rm -f "$tmp"
    docker exec "$ISAAC_SIM_CONTAINER_NAME" chmod +x "$LAUNCH_SCRIPT_PATH" >/dev/null 2>&1 || true
}

svc_start() {
    require_sim_container || return 1

    if docker exec "$ISAAC_SIM_CONTAINER_NAME" pgrep -f 'zed_sim_publisher' >/dev/null 2>&1; then
        log_ok "already running"
        return 0
    fi

    kill_pattern_in_container "$PATTERN" 5 2>/dev/null || true
    write_launch_script

    local env_args=()
    for v in ZED_CAMERA_MODEL ZED_CAMERA_NAME ZED_GRAB_RESOLUTION \
        ZED_DEPTH_MODE ISAAC_SIM_HEADLESS ISAAC_SIM_WORLD \
        ISAAC_SIM_DRONE_START_X ISAAC_SIM_DRONE_START_Y \
        ISAAC_SIM_DRONE_START_Z ISAAC_SIM_DRONE_START_YAW; do
        env_args+=("-e" "${v}=${!v:-}")
    done

    log_info "launching ZED sim publisher inside $ISAAC_SIM_CONTAINER_NAME"
    docker exec "${env_args[@]}" -d "$ISAAC_SIM_CONTAINER_NAME" \
        bash -c "nohup bash $LAUNCH_SCRIPT_PATH > $LAUNCH_LOG 2>&1 &"

    if wait_for "docker exec $ISAAC_SIM_CONTAINER_NAME bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null; timeout 8 ros2 topic info --no-daemon /zed/zed_node/odom 2>/dev/null | grep -Eq \"Publisher count: [1-9]\"'" 60; then
        log_ok "ZED sim publishing odometry"
        return 0
    fi
    log_warn "ZED sim did not publish odometry within 60s (check: nomad logs isaac_sim_zed)"
    return 1
}

svc_stop() {
    if ! sim_container_running; then
        log_ok "container not running; nothing to stop"
        return 0
    fi
    log_info "stopping ZED sim publisher"
    kill_pattern_in_container "$PATTERN" 5 2>/dev/null || true
    log_ok "stopped"
}

svc_status() {
    if ! sim_container_running; then
        log_warn "container not running"
        return 1
    fi
    status_pattern_in_container 'zed_sim_publisher'
}

svc_logs() {
    require_sim_container || return 1
    docker exec "$ISAAC_SIM_CONTAINER_NAME" tail -n 100 -F "$LAUNCH_LOG"
}

dispatch_service "$@"
