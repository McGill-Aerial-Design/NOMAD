# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# shellcheck shell=bash
# =============================================================================
# scripts/lib/common.sh — shared helpers for the NOMAD service scripts.
#
# Source from each scripts/services/*.sh file. Do not execute directly.
#
# Provides:
#   - Logging (log_info / log_ok / log_warn / log_fail)
#   - load_nomad_env       Load config/nomad.env into the environment
#   - in_container <cmd>   Run a command inside the Isaac ROS container
#   - container_running    Returns 0 if the Isaac ROS container is up
#   - require_container    Aborts if the Isaac ROS container is not up
#   - kill_pattern_host    pkill a pattern on the host with a wait+SIGKILL fallback
#   - kill_pattern_in_container  Same, but inside the container
#   - wait_for <cmd> <timeout>   Wait until <cmd> exits 0
# =============================================================================

# Idempotent guard so multiple `source` calls are cheap and safe.
if [ -n "${_NOMAD_COMMON_SH_LOADED:-}" ]; then
    return 0 2>/dev/null || exit 0
fi
_NOMAD_COMMON_SH_LOADED=1

# ---------------------------------------------------------------------------
# Repo / config paths
# ---------------------------------------------------------------------------
# scripts/lib/common.sh -> scripts/lib -> scripts -> repo root
_NOMAD_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NOMAD_REPO_ROOT="${NOMAD_REPO_ROOT:-$(cd "$_NOMAD_LIB_DIR/../.." && pwd)}"
NOMAD_ENV_FILE="${NOMAD_ENV_FILE:-$NOMAD_REPO_ROOT/config/nomad.env}"

# ---------------------------------------------------------------------------
# Colors / logging
# ---------------------------------------------------------------------------
if [ -t 1 ]; then
    _C_RED='\033[0;31m'; _C_GRN='\033[0;32m'; _C_YEL='\033[1;33m'
    _C_BLU='\033[0;34m'; _C_OFF='\033[0m'
else
    _C_RED=''; _C_GRN=''; _C_YEL=''; _C_BLU=''; _C_OFF=''
fi

# Service name (overridable via SERVICE env or set_service helper).
SERVICE="${SERVICE:-nomad}"

_log()      { printf "%b[%s] %s%b %s\n" "$1" "$SERVICE" "$2" "$_C_OFF" "$3"; }
log_info()  { _log "$_C_BLU" "INFO" "$1"; }
log_ok()    { _log "$_C_GRN" "OK"   "$1"; }
log_warn()  { _log "$_C_YEL" "WARN" "$1"; }
log_fail()  { _log "$_C_RED" "FAIL" "$1" >&2; }

# ---------------------------------------------------------------------------
# Env loader
# ---------------------------------------------------------------------------
load_nomad_env() {
    if [ ! -f "$NOMAD_ENV_FILE" ]; then
        log_fail "Config not found: $NOMAD_ENV_FILE"
        log_fail "Copy config/nomad.env into place and edit for this deployment."
        return 1
    fi
    set -a
    # shellcheck disable=SC1090
    . "$NOMAD_ENV_FILE"
    set +a
    mkdir -p "${NOMAD_LOG_DIR:-/tmp}" "${NOMAD_RUN_DIR:-/tmp}" 2>/dev/null || true
}

# ---------------------------------------------------------------------------
# Autostart predicate. Returns 0 if NOMAD_AUTOSTART_<KEY>=true.
# Usage: if autostart_enabled VIDEO_BRIDGE; then ...; fi
# ---------------------------------------------------------------------------
autostart_enabled() {
    local key="NOMAD_AUTOSTART_$1"
    local val="${!key:-false}"
    [ "$val" = "true" ] || [ "$val" = "1" ]
}

# ---------------------------------------------------------------------------
# Container helpers
# ---------------------------------------------------------------------------
container_running() {
    [ -n "${ISAAC_CONTAINER_NAME:-}" ] || return 1
    docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$ISAAC_CONTAINER_NAME"
}

require_container() {
    if ! container_running; then
        log_fail "Isaac ROS container '${ISAAC_CONTAINER_NAME}' is not running."
        log_fail "Start it with: nomad start isaac_ros_container"
        return 1
    fi
}

# in_container <bash command string>
in_container() {
    docker exec "$ISAAC_CONTAINER_NAME" bash -c "$1"
}

# in_container_detached <bash command string>
in_container_detached() {
    docker exec -d "$ISAAC_CONTAINER_NAME" bash -c "$1"
}

# Standard ROS2 sourcing prelude for in-container commands.
ros_setup_prelude() {
    local ros_root="${NOMAD_ROS_ROOT:-/opt/ros/humble}"
    local workspace="${NOMAD_ISAAC_WORKSPACE:-/workspaces/isaac_ros-dev}"
    cat <<EOS
GXF_LIB_DIRS=\$(find ${ros_root}/share -path '*/gxf/lib' -type d 2>/dev/null | tr '\n' ':')
export LD_LIBRARY_PATH=${ros_root}/lib:${ros_root}/lib/aarch64-linux-gnu:/usr/local/zed/lib:\${GXF_LIB_DIRS}\${LD_LIBRARY_PATH:-}
source ${ros_root}/install/setup.bash 2>/dev/null || source ${ros_root}/setup.bash 2>/dev/null
source ${workspace}/install/setup.bash 2>/dev/null || true
# NOMAD C++ core + nomad_ros adapter workspace (built into the jetson /
# sim-isaac / sim-ros images; absent on host-only systems).
source /ws/install/setup.bash 2>/dev/null || true
export EGL_PLATFORM=device
EOS
}

# ---------------------------------------------------------------------------
# Process control. A service script must ONLY kill its own patterns —
# never another service's. The pattern should be specific enough to match
# only this service.
# ---------------------------------------------------------------------------
_kill_loop() {
    local where="$1"        # "host" or "container"
    local pattern="$2"
    local timeout="${3:-5}"
    local cmd_prefix=""
    [ "$where" = "container" ] && cmd_prefix="docker exec $ISAAC_CONTAINER_NAME"

    $cmd_prefix pkill -f "$pattern" 2>/dev/null || true
    local i=0
    while [ $i -lt $((timeout * 2)) ]; do
        if ! $cmd_prefix pgrep -f "$pattern" >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.5
        i=$((i + 1))
    done
    $cmd_prefix pkill -9 -f "$pattern" 2>/dev/null || true
    sleep 0.5
}

kill_pattern_host()         { _kill_loop host      "$1" "${2:-5}"; }
kill_pattern_in_container() { _kill_loop container "$1" "${2:-5}"; }

# ---------------------------------------------------------------------------
# wait_for "<shell test command>" <timeout-seconds>
# ---------------------------------------------------------------------------
wait_for() {
    local check="$1"
    local timeout="${2:-30}"
    local i=0
    while [ $i -lt "$timeout" ]; do
        if bash -c "$check" >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
        i=$((i + 1))
    done
    return 1
}

# ---------------------------------------------------------------------------
# Generic process-status helper for host processes matched by a pattern.
# Echoes the PID(s) (or "(not running)") and returns 0/1 accordingly.
# ---------------------------------------------------------------------------
status_pattern_host() {
    local pattern="$1"
    local pids
    pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    if [ -n "$pids" ]; then
        log_ok "running (PID: $(echo "$pids" | tr '\n' ' '))"
        return 0
    fi
    log_warn "not running"
    return 1
}

status_pattern_in_container() {
    local pattern="$1"
    if ! container_running; then
        log_warn "container '$ISAAC_CONTAINER_NAME' not running"
        return 1
    fi
    local pids
    pids=$(docker exec "$ISAAC_CONTAINER_NAME" pgrep -f "$pattern" 2>/dev/null || true)
    if [ -n "$pids" ]; then
        log_ok "running in container (PID: $(echo "$pids" | tr '\n' ' '))"
        return 0
    fi
    log_warn "not running"
    return 1
}

# ---------------------------------------------------------------------------
# Standard service-script entrypoint.
#
# Each service script defines four functions (svc_start, svc_stop, svc_status,
# svc_logs) and then calls `dispatch_service "$@"`.
# ---------------------------------------------------------------------------
dispatch_service() {
    load_nomad_env || exit 1
    local action="${1:-status}"
    case "$action" in
        start)   svc_start ;;
        stop)    svc_stop ;;
        restart) svc_stop; sleep 1; svc_start ;;
        status)  svc_status ;;
        logs)    svc_logs "${2:-}" ;;
        *)
            echo "Usage: $0 {start|stop|restart|status|logs}" >&2
            exit 2
            ;;
    esac
}
