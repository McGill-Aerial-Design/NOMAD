#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# nomad-ros-vehicle service
#
# Owns: the read-only C++ nomad_vehicle_node telemetry observer inside the
# Isaac ROS container. Depends on: isaac_ros_container.
#
# Requires the Isaac ROS container image rebuilt with the MAVSDK observer and
# nomad_ros adapter (docker/Dockerfile.jetson, colcon workspace /ws/install).
# =============================================================================
set -u
SERVICE="ros-vehicle"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
. "$SCRIPT_DIR/../lib/common.sh"

PATTERN='nomad_vehicle_node'
PROC_MATCH='nomad_vehicle_node'
LAUNCH_SCRIPT_PATH=/tmp/nomad_ros_vehicle_launch.sh
LAUNCH_LOG=/tmp/nomad_ros_vehicle.log

write_launch_script() {
    local tmp
    tmp=$(mktemp)
    {
        echo "#!/bin/bash"
        # No `set -u`: ROS2's setup.bash isn't -u clean and would abort silently.
        ros_setup_prelude
        cat <<'EOS'
# Run the C++ observer from the image's colcon workspace (/ws/install,
# sourced by ros_setup_prelude). mavlink-router's nav_bridge leg feeds a
# receive-only telemetry stream on UDP 14552.
ARGS=(
    --ros-args
    -p observation_endpoint:=udpin:0.0.0.0:${NOMAD_ROS_OBSERVATION_PORT:-14552}
    -p expected_system_id:=${NOMAD_ROS_EXPECTED_SYSTEM_ID:-1}
    -p publish_rate_hz:=${NOMAD_ROS_PUBLISH_RATE_HZ:-10.0}
)

# Restart loop: keeps the node up through transient crashes.
# Trap SIGTERM so `docker exec ... pkill` cleanly stops the wrapper.
trap 'kill -TERM "$NODE_PID" 2>/dev/null; exit 0' TERM INT
while true; do
    ros2 run nomad_ros nomad_vehicle_node "${ARGS[@]}" &
    NODE_PID=$!
    wait "$NODE_PID"
    rc=$?
    echo "[ros-observer] exited rc=$rc, restarting in 10s" >&2
    sleep 10
done
EOS
    } > "$tmp"
    docker cp "$tmp" "$ISAAC_CONTAINER_NAME:$LAUNCH_SCRIPT_PATH" >/dev/null
    rm -f "$tmp"
    in_container "chmod +x $LAUNCH_SCRIPT_PATH"
}

svc_start() {
    require_container || return 1

    if docker exec "$ISAAC_CONTAINER_NAME" pgrep -f "$PROC_MATCH" >/dev/null 2>&1; then
        log_ok "already running"
        return 0
    fi

    kill_pattern_in_container "$PATTERN" 5
    write_launch_script

    local env_args=(
        "-e" "NOMAD_ROS_OBSERVATION_PORT=${NOMAD_ROS_OBSERVATION_PORT:-14552}"
        "-e" "NOMAD_ROS_EXPECTED_SYSTEM_ID=${NOMAD_ROS_EXPECTED_SYSTEM_ID:-1}"
        "-e" "NOMAD_ROS_PUBLISH_RATE_HZ=${NOMAD_ROS_PUBLISH_RATE_HZ:-10.0}"
    )

    log_info "starting node in container"
    docker exec "${env_args[@]}" -d "$ISAAC_CONTAINER_NAME" \
        bash -c "nohup bash $LAUNCH_SCRIPT_PATH > $LAUNCH_LOG 2>&1 &"

    if wait_for "docker exec $ISAAC_CONTAINER_NAME pgrep -f '$PROC_MATCH'" 30; then
        log_ok "node process up"
        return 0
    fi
    log_fail "node did not appear within 30s (check: nomad logs ros_vehicle)"
    return 1
}

svc_stop() {
    if ! container_running; then
        log_ok "container not running; nothing to stop"
        return 0
    fi
    log_info "stopping node"
    kill_pattern_in_container "$PATTERN" 8
    log_ok "stopped"
}

svc_status() { status_pattern_in_container "$PROC_MATCH"; }

svc_logs() {
    require_container || return 1
    docker exec "$ISAAC_CONTAINER_NAME" tail -n 100 -F "$LAUNCH_LOG"
}

dispatch_service "$@"
