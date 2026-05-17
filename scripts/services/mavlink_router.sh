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
EMPTY_CONF_DEFAULT="${NOMAD_LOG_DIR:-/tmp}/mavlink-router-empty.conf"

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

build_gcs_endpoints() {
    local primary="$1"
    local endpoints=()
    local seen=" "
    local all_ips="$primary ${GCS_EXTRA_IPS:-}"
    local ip

    all_ips="${all_ips//,/ }"
    for ip in $all_ips; do
        [ -n "$ip" ] || continue
        case "$seen" in
            *" $ip "*) continue ;;
        esac
        seen="${seen}${ip} "
        endpoints+=("-e" "$ip:$GCS_PORT_LTE")
    done

    endpoints+=("-e" "127.0.0.1:$GCS_PORT_LOCAL")
    printf '%s\n' "${endpoints[@]}"
}

svc_start() {
    if pgrep -x "$PATTERN" >/dev/null 2>&1; then
        log_ok "already running"
        return 0
    fi
    if [ ! -e "$MAVLINK_UART_DEV" ]; then
        # Not a software failure — the FC just isn't plugged in. Exit 0 so the
        # unit becomes "active (exited)" with a warning in the journal, instead
        # of polluting nomad.target with a hardware-state-driven failure.
        # When the FC is connected, `nomad restart mavlink_router` brings it up.
        log_warn "CubePilot not present at $MAVLINK_UART_DEV; staying down (plug in and restart this unit)"
        return 0
    fi
    local gcs
    gcs="$(discover_gcs_ip)"
    local endpoints
    mapfile -t endpoints < <(build_gcs_endpoints "$gcs")
    local empty_conf="${MAVLINK_ROUTER_EMPTY_CONF:-$EMPTY_CONF_DEFAULT}"
    mkdir -p "$(dirname "$empty_conf")"
    cat > "$empty_conf" <<EOF
[General]
TcpServerPort=5760
ReportStats=true
MavlinkDialect=ardupilotmega
EOF
    local attempts delay attempt
    attempts="${MAVLINK_ROUTER_START_ATTEMPTS:-10}"
    delay="${MAVLINK_ROUTER_START_RETRY_DELAY:-2}"
    for attempt in $(seq 1 "$attempts"); do
        log_info "starting (attempt $attempt/$attempts, endpoints: ${endpoints[*]})"
        nohup mavlink-routerd \
            -c "$empty_conf" \
            "${endpoints[@]}" \
            "$MAVLINK_UART_DEV:${MAVLINK_UART_BAUD:-921600}" \
            > "$LOG_FILE_DEFAULT" 2>&1 &
        sleep 2
        if pgrep -x "$PATTERN" >/dev/null 2>&1; then
            log_ok "started"
            return 0
        fi
        if [ "$attempt" -lt "$attempts" ]; then
            log_warn "start attempt $attempt failed; retrying in ${delay}s (see $LOG_FILE_DEFAULT)"
            sleep "$delay"
        fi
    done
    log_fail "failed to start; see $LOG_FILE_DEFAULT"
    return 1
}

svc_stop() {
    log_info "stopping"
    pkill -x "$PATTERN" 2>/dev/null || true
    local i=0
    while [ $i -lt 10 ]; do
        if ! pgrep -x "$PATTERN" >/dev/null 2>&1; then
            log_ok "stopped"
            return 0
        fi
        sleep 0.5
        i=$((i + 1))
    done
    pkill -9 -x "$PATTERN" 2>/dev/null || true
    log_ok "stopped"
}

svc_status() {
    local pids
    pids=$(pgrep -x "$PATTERN" 2>/dev/null || true)
    if [ -n "$pids" ]; then
        log_ok "running (PID: $(echo "$pids" | tr '\n' ' '))"
        return 0
    fi
    log_warn "not running"
    return 1
}

svc_logs() {
    [ -f "$LOG_FILE_DEFAULT" ] && tail -n 50 -F "$LOG_FILE_DEFAULT" || \
        log_warn "no log file (running under systemd? use: journalctl -u nomad-mavlink-router -f)"
}

dispatch_service "$@"
