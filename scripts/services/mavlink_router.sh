#!/bin/bash
# =============================================================================
# nomad-mavlink-router service
# Owns: mavlink-routerd process on the host.
# =============================================================================
set -u
SERVICE="mavlink-router"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../lib/common.sh
. "$SCRIPT_DIR/../lib/common.sh"

PATTERN='mavlink-routerd'
LOG_FILE_DEFAULT="${NOMAD_LOG_DIR:-/tmp}/mavlink.log"

discover_gcs_ip() {
    if [ -n "${GCS_IP:-}" ]; then
        echo "$GCS_IP"
        return
    fi
    # Pick first non-self Tailscale peer.
    local ip
    ip=$(tailscale status 2>/dev/null | grep -v "$(hostname)" \
            | grep -oE '[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' | head -1 || true)
    echo "${ip:-192.168.1.255}"
}

svc_start() {
    if pgrep -f "$PATTERN" >/dev/null 2>&1; then
        log_ok "already running"
        return 0
    fi
    if [ ! -e "$MAVLINK_UART_DEV" ]; then
        log_warn "CubePilot not present at $MAVLINK_UART_DEV; refusing to start"
        return 1
    fi
    local gcs
    gcs="$(discover_gcs_ip)"
    log_info "starting (GCS: $gcs:$GCS_PORT_LTE, local: 127.0.0.1:$GCS_PORT_LOCAL)"
    nohup mavlink-routerd \
        -e "$gcs:$GCS_PORT_LTE" \
        -e "127.0.0.1:$GCS_PORT_LOCAL" \
        "$MAVLINK_UART_DEV" \
        > "$LOG_FILE_DEFAULT" 2>&1 &
    sleep 2
    if pgrep -f "$PATTERN" >/dev/null 2>&1; then
        log_ok "started"
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
        log_warn "no log file (running under systemd? use: journalctl -u nomad-mavlink-router -f)"
}

dispatch_service "$@"
