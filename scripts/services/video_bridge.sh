#!/bin/bash
# =============================================================================
# nomad-video-bridge service
#
# Owns: simple_video_bridge.py running inside the Isaac ROS container.
#
# Implementation: lifecycle is driven via Edge Core's existing
# /api/video/start and /api/video/stop endpoints — Edge Core's
# VideoStreamManager handles GStreamer wiring, MediaMTX target, retry, etc.
# This service is the SINGLE OWNER of "should it be running right now."
#
# Edge Core's in-process auto-start thread is disabled (nomad.env:
# NOMAD_VIDEO_AUTO_START=false). The mission planner UI can still call the
# API directly for ad-hoc start/stop; that does not conflict because the API
# is idempotent.
#
# Depends on: edge_core (for the API), isaac_ros_container (to host the bridge
# process). Does NOT require zed_wrapper to be up — but with no ZED topics
# available, Edge Core's start call will return 503 and this script will
# retry.
# =============================================================================
set -u
SERVICE="video-bridge"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
. "$SCRIPT_DIR/../lib/common.sh"

PATTERN_IN_CONTAINER='simple_video_bridge'

api_curl() {
    local args=(-sS --max-time 8)
    [ -n "${NOMAD_API_KEY:-}" ] && args+=(-H "X-API-Key: $NOMAD_API_KEY")
    curl "${args[@]}" "$@"
}

svc_start() {
    if ! wait_for "curl -sS --max-time 1 $NOMAD_API_URL/health" 30; then
        log_fail "Edge Core API not reachable at $NOMAD_API_URL; start it first"
        return 1
    fi
    if ! container_running; then
        log_fail "Isaac ROS container not running; start it first"
        return 1
    fi

    # Already streaming? Idempotent.
    if api_curl "$NOMAD_API_URL/api/video/status" | grep -q '"streaming":true'; then
        log_ok "already streaming"
        return 0
    fi

    local attempts=5
    local i=1
    while [ $i -le $attempts ]; do
        log_info "POST /api/video/start (attempt $i/$attempts)"
        local body
        body=$(api_curl -X POST "$NOMAD_API_URL/api/video/start" || true)
        if echo "$body" | grep -q '"success":true'; then
            log_ok "streaming on rtsp://<jetson-ip>:$RTSP_PORT/$VIDEO_BRIDGE_STREAM_PATH"
            return 0
        fi
        log_warn "Edge Core: $body"
        sleep 5
        i=$((i + 1))
    done
    log_fail "video bridge did not start after $attempts attempts"
    return 1
}

svc_stop() {
    if curl -sS --max-time 2 "$NOMAD_API_URL/health" >/dev/null 2>&1; then
        log_info "POST /api/video/stop"
        api_curl -X POST "$NOMAD_API_URL/api/video/stop" >/dev/null 2>&1 || true
    fi
    # Belt-and-braces in case the API was down: kill the in-container process.
    if container_running; then
        kill_pattern_in_container "$PATTERN_IN_CONTAINER" 5
    fi
    log_ok "stopped"
}

svc_status() {
    if ! curl -sS --max-time 2 "$NOMAD_API_URL/health" >/dev/null 2>&1; then
        log_warn "Edge Core API unreachable"
        return 1
    fi
    local body
    body=$(api_curl "$NOMAD_API_URL/api/video/status" || true)
    if echo "$body" | grep -q '"streaming":true'; then
        log_ok "streaming"
        return 0
    fi
    log_warn "not streaming ($body)"
    return 1
}

svc_logs() {
    if ! container_running; then
        log_warn "container not running"
        return 1
    fi
    docker exec "$ISAAC_CONTAINER_NAME" \
        bash -c 'tail -n 100 -F /tmp/video_bridge.log 2>/dev/null || tail -n 100 -F /tmp/simple_video_bridge.log'
}

dispatch_service "$@"
