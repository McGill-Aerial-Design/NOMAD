#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
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

# The bridge is a Python package run as a module (relative imports), so the
# process command line contains "nomad_ros_http_bridge.main".
PATTERN='nomad_ros_http_bridge\.main|nomad_ros_http_bridge_launch'
PROC_MATCH='nomad_ros_http_bridge\.main'
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
# Run the bridge as a package module so its relative imports resolve. The
# package dir is /tmp/nomad_ros_http_bridge (copied via docker cp below), so
# /tmp must be on PYTHONPATH and is also used as the working directory.
export PYTHONPATH=/tmp:/workspaces/isaac_ros-dev:${PYTHONPATH:-}
cd /tmp || exit 1

ARGS=(
    --host localhost
    --port "${NOMAD_API_PORT}"
    --rate "${ROS_HTTP_BRIDGE_RATE}"
    --vio-topic "${ROS_HTTP_BRIDGE_VIO_TOPIC}"
    --mag-topic "${ROS_HTTP_BRIDGE_MAG_TOPIC}"
)

case "${NOMAD_AUTOSTART_NVBLOX:-false}:${NOMAD_ENABLE_NVBLOX_MESH:-false}" in
    true:*|TRUE:*|1:*|yes:*|YES:*|on:*|ON:*|*:true|*:TRUE|*:1|*:yes|*:YES|*:on|*:ON)
        ARGS+=(--mesh-topic /nvblox_node/color_layer_marker)
        ;;
    *)
        ARGS+=(--disable-mesh)
        ;;
esac

# Restart loop: keeps VIO telemetry flowing through transient crashes.
# Trap SIGTERM so `docker exec ... pkill` cleanly stops the wrapper.
trap 'kill -TERM "$BRIDGE_PID" 2>/dev/null; exit 0' TERM INT
while true; do
    python3 -m nomad_ros_http_bridge.main "${ARGS[@]}" &
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

    if docker exec "$ISAAC_CONTAINER_NAME" pgrep -f "$PROC_MATCH" >/dev/null 2>&1; then
        log_ok "already running"
        return 0
    fi

    kill_pattern_in_container "$PATTERN" 5
    docker exec "$ISAAC_CONTAINER_NAME" mkdir -p /tmp/nomad_ros_http_bridge
    docker cp "$NOMAD_REPO_ROOT/edge_core/ros_http_bridge/." \
        "$ISAAC_CONTAINER_NAME:/tmp/nomad_ros_http_bridge/" >/dev/null
    write_launch_script

    local env_args=(
        "-e" "NOMAD_API_PORT=$NOMAD_API_PORT"
        "-e" "ROS_HTTP_BRIDGE_RATE=$ROS_HTTP_BRIDGE_RATE"
        "-e" "ROS_HTTP_BRIDGE_VIO_TOPIC=$ROS_HTTP_BRIDGE_VIO_TOPIC"
        "-e" "ROS_HTTP_BRIDGE_MAG_TOPIC=$ROS_HTTP_BRIDGE_MAG_TOPIC"
        "-e" "NOMAD_AUTOSTART_NVBLOX=${NOMAD_AUTOSTART_NVBLOX:-false}"
        "-e" "NOMAD_ENABLE_NVBLOX_MESH=${NOMAD_ENABLE_NVBLOX_MESH:-false}"
    )
    [ -n "${NOMAD_API_KEY:-}" ]        && env_args+=("-e" "NOMAD_API_KEY=$NOMAD_API_KEY")
    [ -n "${NOMAD_INTERNAL_TOKEN:-}" ] && env_args+=("-e" "NOMAD_INTERNAL_TOKEN=$NOMAD_INTERNAL_TOKEN")

    log_info "starting bridge in container"
    docker exec "${env_args[@]}" -d "$ISAAC_CONTAINER_NAME" \
        bash -c "nohup bash $LAUNCH_SCRIPT_PATH > $LAUNCH_LOG 2>&1 &"

    if wait_for "docker exec $ISAAC_CONTAINER_NAME pgrep -f '$PROC_MATCH'" 30; then
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

svc_status() { status_pattern_in_container "$PROC_MATCH"; }

svc_logs() {
    require_container || return 1
    docker exec "$ISAAC_CONTAINER_NAME" tail -n 100 -F "$LAUNCH_LOG"
}

dispatch_service "$@"
