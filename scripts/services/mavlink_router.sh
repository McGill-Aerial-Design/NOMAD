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
RUN_PATTERN='mavlink_router.sh run'
LOG_FILE_DEFAULT=""
EMPTY_CONF_DEFAULT=""
RUN_PID_FILE=""
DEFAULT_MAVLINK_UART_DEV="/dev/ttyACM0"

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

discover_mavlink_dev() {
    local configured="${MAVLINK_UART_DEV:-$DEFAULT_MAVLINK_UART_DEV}"

    # If a deployment pins a non-default device path, honor it when present.
    if [ "$configured" != "$DEFAULT_MAVLINK_UART_DEV" ] && [ -e "$configured" ]; then
        echo "$configured"
        return 0
    fi

    # Cube Orange enumerates as two ACM interfaces. Use interface 00 for MAVLink.
    local by_id
    by_id=$(find /dev/serial/by-id -maxdepth 1 -type l \
        \( -iname '*CubePilot*if00*' -o -iname '*CubeOrange*if00*' -o -iname '*CubeOrange+*if00*' \) \
        2>/dev/null | sort | head -n 1 || true)
    if [ -n "$by_id" ] && [ -e "$by_id" ]; then
        echo "$by_id"
        return 0
    fi

    if [ -e "$configured" ]; then
        echo "$configured"
        return 0
    fi

    local acm
    acm=$(find /dev -maxdepth 1 -name 'ttyACM*' -type c 2>/dev/null | sort | head -n 1 || true)
    if [ -n "$acm" ]; then
        echo "$acm"
        return 0
    fi

    return 1
}

write_empty_conf() {
    local empty_conf="${MAVLINK_ROUTER_EMPTY_CONF:-$EMPTY_CONF_DEFAULT}"
    mkdir -p "$(dirname "$empty_conf")"
    cat > "$empty_conf" <<EOF
[General]
TcpServerPort=5760
ReportStats=true
MavlinkDialect=ardupilotmega
EOF
    echo "$empty_conf"
}

svc_run() {
    local retry_delay="${MAVLINK_ROUTER_START_RETRY_DELAY:-2}"
    echo "$$" > "$RUN_PID_FILE"
    trap 'rm -f "$RUN_PID_FILE"; pkill -x "$PATTERN" 2>/dev/null || true; exit 0' TERM INT EXIT
    while true; do
        local mavlink_dev=""
        if ! mavlink_dev="$(discover_mavlink_dev)"; then
            log_warn "CubePilot not present; waiting for USB serial device"
            sleep "$retry_delay"
            continue
        fi

        local gcs endpoints empty_conf
        gcs="$(discover_gcs_ip)"
        mapfile -t endpoints < <(build_gcs_endpoints "$gcs")
        empty_conf="$(write_empty_conf)"

        log_info "starting foreground router"
        mavlink-routerd \
            -c "$empty_conf" \
            "${endpoints[@]}" \
            "$mavlink_dev:${MAVLINK_UART_BAUD:-921600}"

        local rc=$?
        log_warn "mavlink-routerd exited with status $rc; rediscovering Cube USB in ${retry_delay}s"
        sleep "$retry_delay"
    done
}

svc_start() {
    local run_pid=""
    [ -f "$RUN_PID_FILE" ] && run_pid="$(cat "$RUN_PID_FILE" 2>/dev/null || true)"
    if [ -n "$run_pid" ] && ps -p "$run_pid" -o args= 2>/dev/null | grep -q "$RUN_PATTERN"; then
        log_ok "supervisor already running"
        return 0
    fi
    if pgrep -x "$PATTERN" >/dev/null 2>&1; then
        log_ok "router already running"
        return 0
    fi
    mkdir -p "$(dirname "$LOG_FILE_DEFAULT")"
    log_info "starting supervisor"
    nohup "$0" run > "$LOG_FILE_DEFAULT" 2>&1 &
    sleep 2
    run_pid="$(cat "$RUN_PID_FILE" 2>/dev/null || true)"
    if [ -n "$run_pid" ] && ps -p "$run_pid" -o args= 2>/dev/null | grep -q "$RUN_PATTERN"; then
        log_ok "supervisor started"
        return 0
    fi
    log_fail "failed to start supervisor; see $LOG_FILE_DEFAULT"
    return 1
}

svc_stop() {
    log_info "stopping"
    local run_pid=""
    [ -f "$RUN_PID_FILE" ] && run_pid="$(cat "$RUN_PID_FILE" 2>/dev/null || true)"
    if [ -n "$run_pid" ]; then
        kill "$run_pid" 2>/dev/null || true
    fi
    pkill -f "$RUN_PATTERN" 2>/dev/null || true
    pkill -x "$PATTERN" 2>/dev/null || true
    local i=0
    while [ $i -lt 10 ]; do
        run_pid="$(cat "$RUN_PID_FILE" 2>/dev/null || true)"
        if { [ -z "$run_pid" ] || ! ps -p "$run_pid" >/dev/null 2>&1; } && ! pgrep -x "$PATTERN" >/dev/null 2>&1; then
            rm -f "$RUN_PID_FILE"
            log_ok "stopped"
            return 0
        fi
        sleep 0.5
        i=$((i + 1))
    done
    pkill -9 -f "$RUN_PATTERN" 2>/dev/null || true
    pkill -9 -x "$PATTERN" 2>/dev/null || true
    rm -f "$RUN_PID_FILE"
    log_ok "stopped"
}

svc_status() {
    local pids
    local run_pid=""
    pids=$(pgrep -x "$PATTERN" 2>/dev/null || true)
    [ -f "$RUN_PID_FILE" ] && run_pid="$(cat "$RUN_PID_FILE" 2>/dev/null || true)"
    if [ -n "$pids" ] || { [ -n "$run_pid" ] && ps -p "$run_pid" >/dev/null 2>&1; }; then
        log_ok "running (supervisor: ${run_pid:-none}, router: $(echo "${pids:-none}" | tr '\n' ' '))"
        return 0
    fi
    log_warn "not running"
    return 1
}

svc_logs() {
    [ -f "$LOG_FILE_DEFAULT" ] && tail -n 50 -F "$LOG_FILE_DEFAULT" || \
        log_warn "no log file (running under systemd? use: journalctl -u nomad-mavlink-router -f)"
}

load_nomad_env || exit 1
LOG_FILE_DEFAULT="${MAVLINK_ROUTER_LOG_FILE:-${NOMAD_LOG_DIR:-/tmp}/mavlink.log}"
EMPTY_CONF_DEFAULT="${MAVLINK_ROUTER_EMPTY_CONF:-${NOMAD_LOG_DIR:-/tmp}/mavlink-router-empty.conf}"
RUN_PID_FILE="${MAVLINK_ROUTER_PID_FILE:-${NOMAD_RUN_DIR:-/tmp}/mavlink-router-supervisor.pid}"
if ! mkdir -p "$(dirname "$RUN_PID_FILE")" 2>/dev/null || ! touch "$RUN_PID_FILE" 2>/dev/null; then
    RUN_PID_FILE="/tmp/mavlink-router-supervisor.pid"
    touch "$RUN_PID_FILE" 2>/dev/null || true
fi
case "${1:-status}" in
    run)     svc_run ;;
    start)   svc_start ;;
    stop)    svc_stop ;;
    restart) svc_stop; sleep 1; svc_start ;;
    status)  svc_status ;;
    logs)    svc_logs "${2:-}" ;;
    *)
        echo "Usage: $0 {run|start|stop|restart|status|logs}" >&2
        exit 2
        ;;
esac
