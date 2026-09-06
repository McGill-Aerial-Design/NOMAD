#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# nomad-ros-vehicle service
#
# Owns: the C++ nomad_vehicle_node (ros2 run nomad_ros) inside the Isaac ROS
# container. Depends on: isaac_ros_container.
#
# Does NOT require the ZED wrapper to be up — the node simply has no VIO
# health/confidence to gate velocity commands until ZED publishes. This is
# intentional decoupling.
#
# Requires the Isaac ROS container image rebuilt with the C++ core + nomad_ros
# adapter (docker/Dockerfile.jetson, colcon workspace /ws/install).
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
# Run the C++ adapter node from the image's colcon workspace (/ws/install,
# sourced by ros_setup_prelude). The node binds one MAVLink UDP link —
# mavlink-router's nav_bridge leg (127.0.0.1:14552) feeds it.
ARGS=(
    --ros-args
    -p endpoint:=udpin:0.0.0.0:${NOMAD_ROS_MAVLINK_PORT:-14552}
    -p publish_rate_hz:=${NOMAD_ROS_PUBLISH_RATE_HZ:-10.0}
    -p min_vio_confidence:=${NOMAD_ROS_MIN_VIO_CONFIDENCE:-0.3}
    -p vio_timeout_ms:=${NOMAD_ROS_VIO_TIMEOUT_MS:-1000}
    -p command_timeout_ms:=${NOMAD_ROS_COMMAND_TIMEOUT_MS:-500}
)

# Restart loop: keeps the node up through transient crashes.
# Trap SIGTERM so `docker exec ... pkill` cleanly stops the wrapper.
trap 'kill -TERM "$NODE_PID" 2>/dev/null; exit 0' TERM INT
while true; do
    ros2 run nomad_ros nomad_vehicle_node "${ARGS[@]}" &
    NODE_PID=$!
    wait "$NODE_PID"
    rc=$?
    echo "[ros-vehicle] exited rc=$rc, restarting in 10s" >&2
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
        "-e" "NOMAD_ROS_MAVLINK_PORT=${NOMAD_ROS_MAVLINK_PORT:-14552}"
        "-e" "NOMAD_ROS_PUBLISH_RATE_HZ=${NOMAD_ROS_PUBLISH_RATE_HZ:-10.0}"
        "-e" "NOMAD_ROS_MIN_VIO_CONFIDENCE=${NOMAD_ROS_MIN_VIO_CONFIDENCE:-0.3}"
        "-e" "NOMAD_ROS_VIO_TIMEOUT_MS=${NOMAD_ROS_VIO_TIMEOUT_MS:-1000}"
        "-e" "NOMAD_ROS_COMMAND_TIMEOUT_MS=${NOMAD_ROS_COMMAND_TIMEOUT_MS:-500}"
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
