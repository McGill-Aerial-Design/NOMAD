#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# =============================================================================
# nomad-video-bridge service
#
# Owns the optional simple_video_bridge process inside the Isaac ROS container.
# The bridge publishes an ROS image topic to MediaMTX over RTSP. Its local HTTP
# API is bound to loopback by default and is not used for service lifecycle.
# =============================================================================
set -u
SERVICE="video-bridge"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
. "$SCRIPT_DIR/../lib/common.sh"

PID_FILE=/tmp/video_bridge.pid
LAUNCH_SCRIPT_PATH=/tmp/nomad_video_bridge_launch.sh
LAUNCH_LOG=/tmp/video_bridge.log

write_launch_script() {
    local tmp
    tmp=$(mktemp)
    trap 'rm -f "$tmp"' RETURN
    cat > "$tmp" <<'EOS'
#!/bin/bash
set -u

PID_FILE=/tmp/video_bridge.pid
if [ -s "$PID_FILE" ]; then
    old_pid=$(cat "$PID_FILE")
    case "$old_pid" in
        *[!0-9]*|"") old_pid="" ;;
    esac
    if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null && \
        ps -p "$old_pid" -o args= 2>/dev/null | grep -Fq "python3 -m python.tools.simple_video_bridge"; then
        exit 0
    fi
fi

cleanup() {
    rm -f "$PID_FILE"
}

args=(
    --source-topic "${VIDEO_BRIDGE_SOURCE_TOPIC:-/nomad/camera/image}"
    --width "${VIDEO_BRIDGE_WIDTH:-640}"
    --height "${VIDEO_BRIDGE_HEIGHT:-360}"
    --fps "${VIDEO_BRIDGE_FPS:-15}"
    --bitrate "${VIDEO_BRIDGE_BITRATE:-800}"
    --http-host "${VIDEO_RELAY_HTTP_HOST:-127.0.0.1}"
    --http-port "${VIDEO_RELAY_HTTP_PORT:-9200}"
    --rtsp-path "${VIDEO_BRIDGE_STREAM_PATH:-stream}"
)
if [ -n "${NOMAD_VIDEO_RTSP_PUBLISH_URL:-}" ]; then
    args+=(--rtsp-url "$NOMAD_VIDEO_RTSP_PUBLISH_URL")
fi
if [ -n "${NOMAD_VIDEO_FLIP_METHOD:-}" ]; then
    args+=(--flip-method "$NOMAD_VIDEO_FLIP_METHOD")
fi
python3 -m python.tools.simple_video_bridge "${args[@]}" &
bridge_pid=$!
printf '%s\n' "$bridge_pid" > "$PID_FILE"
trap 'kill -TERM "$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true; cleanup' TERM INT EXIT
wait "$bridge_pid"
EOS
    docker cp "$tmp" "$ISAAC_CONTAINER_NAME:$LAUNCH_SCRIPT_PATH" >/dev/null
    in_container "chmod 0755 $LAUNCH_SCRIPT_PATH"
}

bridge_running() {
    docker exec "$ISAAC_CONTAINER_NAME" bash -c '
        [ -s /tmp/video_bridge.pid ] || exit 1
        pid=$(cat /tmp/video_bridge.pid)
        case "$pid" in *[!0-9]*|"") exit 1 ;; esac
        kill -0 "$pid" 2>/dev/null || exit 1
        ps -p "$pid" -o args= 2>/dev/null | grep -Fq "python3 -m python.tools.simple_video_bridge"
    ' >/dev/null 2>&1
}

start_environment_args() {
    printf '%s\0' \
        -e "VIDEO_BRIDGE_SOURCE_TOPIC=${VIDEO_BRIDGE_SOURCE_TOPIC:-/nomad/camera/image}" \
        -e "VIDEO_BRIDGE_WIDTH=${VIDEO_BRIDGE_WIDTH:-640}" \
        -e "VIDEO_BRIDGE_HEIGHT=${VIDEO_BRIDGE_HEIGHT:-360}" \
        -e "VIDEO_BRIDGE_FPS=${VIDEO_BRIDGE_FPS:-15}" \
        -e "VIDEO_BRIDGE_BITRATE=${VIDEO_BRIDGE_BITRATE:-800}" \
        -e "VIDEO_RELAY_HTTP_HOST=${VIDEO_RELAY_HTTP_HOST:-127.0.0.1}" \
        -e "VIDEO_RELAY_HTTP_PORT=${VIDEO_RELAY_HTTP_PORT:-9200}" \
        -e "VIDEO_BRIDGE_STREAM_PATH=${VIDEO_BRIDGE_STREAM_PATH:-stream}" \
        -e "NOMAD_VIDEO_RTSP_PUBLISH_URL=${NOMAD_VIDEO_RTSP_PUBLISH_URL:-}" \
        -e "NOMAD_VIDEO_FLIP_METHOD=${NOMAD_VIDEO_FLIP_METHOD:-}"
}

svc_start() {
    require_container || return 1
    if bridge_running; then
        log_ok "already running"
        return 0
    fi

    write_launch_script || {
        log_fail "could not install the video bridge launch script"
        return 1
    }

    local env_args=()
    while IFS= read -r -d '' arg; do
        env_args+=("$arg")
    done < <(start_environment_args)
    log_info "starting video bridge in container"
    docker exec "${env_args[@]}" -d "$ISAAC_CONTAINER_NAME" \
        bash -c "nohup bash $LAUNCH_SCRIPT_PATH >> $LAUNCH_LOG 2>&1 &"

    if wait_for "docker exec $ISAAC_CONTAINER_NAME test -s $PID_FILE" 15 && bridge_running; then
        log_ok "video bridge process up"
        return 0
    fi
    log_fail "video bridge did not appear within 15s (check: nomad logs video_bridge)"
    return 1
}

svc_stop() {
    if ! container_running; then
        log_ok "container not running; nothing to stop"
        return 0
    fi
    log_info "stopping video bridge"
    docker exec "$ISAAC_CONTAINER_NAME" bash -c '
        if [ -s /tmp/video_bridge.pid ]; then
            pid=$(cat /tmp/video_bridge.pid)
            case "$pid" in *[!0-9]*|"") pid="" ;; esac
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && \
                ps -p "$pid" -o args= 2>/dev/null | grep -Fq "python3 -m python.tools.simple_video_bridge"; then
                kill -TERM "$pid" 2>/dev/null || true
                for _ in 1 2 3 4 5 6 7 8 9 10; do
                    kill -0 "$pid" 2>/dev/null || break
                    sleep 0.5
                done
                kill -KILL "$pid" 2>/dev/null || true
            fi
        fi
        rm -f /tmp/video_bridge.pid
    ' >/dev/null 2>&1 || true
    log_ok "stopped"
}

svc_status() {
    if ! container_running; then
        log_warn "container '$ISAAC_CONTAINER_NAME' not running"
        return 1
    fi
    if bridge_running; then
        log_ok "running in container"
        return 0
    fi
    log_warn "not running (video capability unavailable)"
    return 1
}

svc_logs() {
    require_container || return 1
    docker exec "$ISAAC_CONTAINER_NAME" tail -n 100 -F "$LAUNCH_LOG"
}

dispatch_service "$@"
