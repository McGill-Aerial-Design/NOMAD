#!/bin/bash
# =============================================================================
# nomad-mediamtx service
# Owns: the mediamtx RTSP server process on the host.
# =============================================================================
set -u
SERVICE="mediamtx"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
. "$SCRIPT_DIR/../lib/common.sh"

PATTERN='mediamtx'
LOG_FILE_DEFAULT="${NOMAD_LOG_DIR:-/tmp}/mediamtx.log"

resolve_bin() {
    if command -v mediamtx >/dev/null 2>&1; then
        command -v mediamtx
    elif [ -x "$MEDIAMTX_BIN" ]; then
        echo "$MEDIAMTX_BIN"
    else
        return 1
    fi
}

svc_start() {
    if pgrep -f "$PATTERN" >/dev/null 2>&1; then
        log_ok "already running on rtsp://localhost:$RTSP_PORT"
        return 0
    fi
    local bin
    if ! bin="$(resolve_bin)"; then
        log_fail "mediamtx binary not found (looked in PATH and $MEDIAMTX_BIN)"
        return 1
    fi
    if [ ! -f "$MEDIAMTX_CONFIG" ]; then
        log_fail "config not found: $MEDIAMTX_CONFIG"
        return 1
    fi
    log_info "starting $bin with $MEDIAMTX_CONFIG"
    nohup "$bin" "$MEDIAMTX_CONFIG" > "$LOG_FILE_DEFAULT" 2>&1 &
    sleep 2
    if pgrep -f "$PATTERN" >/dev/null 2>&1; then
        log_ok "ready on rtsp://localhost:$RTSP_PORT"
        return 0
    fi
    log_fail "failed to start; see $LOG_FILE_DEFAULT"
    return 1
}

svc_stop() {
    log_info "stopping"
    kill_pattern_host "$PATTERN" 5
    log_ok "stopped"
}

svc_status() { status_pattern_host "$PATTERN"; }

svc_logs() {
    [ -f "$LOG_FILE_DEFAULT" ] && tail -n 50 -F "$LOG_FILE_DEFAULT" || \
        log_warn "no log file (running under systemd? use: journalctl -u nomad-mediamtx -f)"
}

dispatch_service "$@"
