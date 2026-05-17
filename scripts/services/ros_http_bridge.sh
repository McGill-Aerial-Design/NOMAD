#!/bin/bash
# =============================================================================
# nomad-ros-http-bridge service
#
# Owns: ros_http_bridge.py running inside the Isaac ROS container.
# Depends on: isaac_ros_container.
#
# Does NOT require the ZED wrapper to be up — the bridge will simply have no
# VIO data to forward until ZED publishes. This is intentional decoupling.
# =============================================================================
set -u
SERVICE="ros-http-bridge"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
. "$SCRIPT_DIR/../lib/common.sh"

PATTERN='ros_http_bridge\.py|nomad_ros_http_bridge_launch'
LAUNCH_SCRIPT_PATH=/tmp/nomad_ros_http_bridge_launch.sh
LAUNCH_LOG=/tmp/nomad_ros_http_bridge.log

write_launch_script() {
    local tmp
    tmp=$(mktemp)
    {
        echo "#!/bin/bash"
        # No `set -u`: ROS2's setup.bash isn't -u clean and would abort silently.
        ros_setup_prelude
        cat <<'EOS'
export PYTHONPATH=/workspaces/isaac_ros-dev:${PYTHONPATH:-}

ARGS=(
    --host localhost
    --port "${NOMAD_API_PORT}"
    --rate "${ROS_HTTP_BRIDGE_RATE}"
    --vio-topic "${ROS_HTTP_BRIDGE_VIO_TOPIC}"
    --mag-topic "${ROS_HTTP_BRIDGE_MAG_TOPIC}"
    --high-rate-transport "${ROS_HTTP_BRIDGE_TRANSPORT}"
    --mesh-topic /nvblox_node/color_layer_marker
)

# Restart loop: keeps VIO telemetry flowing through transient crashes.
# Trap SIGTERM so `docker exec ... pkill` cleanly stops the wrapper.
trap 'kill -TERM "$BRIDGE_PID" 2>/dev/null; exit 0' TERM INT
while true; do
    python3 /workspaces/isaac_ros-dev/edge_core/ros_http_bridge.py "${ARGS[@]}" &
    BRIDGE_PID=$!
    wait "$BRIDGE_PID"
    rc=$?
    echo "[ros-http-bridge] exited rc=$rc, restarting in 10s" >&2
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

    if docker exec "$ISAAC_CONTAINER_NAME" pgrep -f 'ros_http_bridge\.py' >/dev/null 2>&1; then
        log_ok "already running"
        return 0
    fi

    kill_pattern_in_container "$PATTERN" 5
    write_launch_script

    local env_args=(
        "-e" "NOMAD_API_PORT=$NOMAD_API_PORT"
        "-e" "ROS_HTTP_BRIDGE_RATE=$ROS_HTTP_BRIDGE_RATE"
        "-e" "ROS_HTTP_BRIDGE_VIO_TOPIC=$ROS_HTTP_BRIDGE_VIO_TOPIC"
        "-e" "ROS_HTTP_BRIDGE_MAG_TOPIC=$ROS_HTTP_BRIDGE_MAG_TOPIC"
        "-e" "ROS_HTTP_BRIDGE_TRANSPORT=$ROS_HTTP_BRIDGE_TRANSPORT"
    )
    [ -n "${NOMAD_API_KEY:-}" ]        && env_args+=("-e" "NOMAD_API_KEY=$NOMAD_API_KEY")
    [ -n "${NOMAD_INTERNAL_TOKEN:-}" ] && env_args+=("-e" "NOMAD_INTERNAL_TOKEN=$NOMAD_INTERNAL_TOKEN")

    log_info "starting bridge in container"
    docker exec "${env_args[@]}" -d "$ISAAC_CONTAINER_NAME" \
        bash -c "nohup bash $LAUNCH_SCRIPT_PATH > $LAUNCH_LOG 2>&1 &"

    if wait_for "docker exec $ISAAC_CONTAINER_NAME pgrep -f 'ros_http_bridge\\.py'" 30; then
        log_ok "bridge process up"
        return 0
    fi
    log_fail "bridge did not appear within 30s (check: nomad logs ros_http_bridge)"
    return 1
}

svc_stop() {
    if ! container_running; then
        log_ok "container not running; nothing to stop"
        return 0
    fi
    log_info "stopping bridge"
    kill_pattern_in_container "$PATTERN" 8
    log_ok "stopped"
}

svc_status() { status_pattern_in_container 'ros_http_bridge\.py'; }

svc_logs() {
    require_container || return 1
    docker exec "$ISAAC_CONTAINER_NAME" tail -n 100 -F "$LAUNCH_LOG"
}

dispatch_service "$@"
