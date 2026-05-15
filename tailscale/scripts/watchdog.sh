#!/bin/bash
# ============================================================
# NOMAD Tailscale Connection Watchdog
# ============================================================
# Monitors Tailscale connection and auto-reconnects if needed.
# Run as a systemd service for continuous monitoring.
#
# Usage:
#   ./watchdog.sh                 # Run manually
#   systemctl start tailscale-watchdog  # Run as service
# ============================================================

# Configuration
HOSTNAME="nomad-jetson"
CHECK_INTERVAL=30          # Seconds between checks
MAX_RETRIES=3              # Max reconnect attempts before giving up
RETRY_DELAY=10             # Seconds between retry attempts
LOG_TAG="tailscale-watchdog"
# Operator user — must match `tailscale set --operator=<user>` on the Jetson
# (defaults to "mad"). When the daemon has any non-default prefs set, a bare
# `tailscale up` refuses with "requires mentioning all non-default flags",
# so we mention them here.
OPERATOR="${TS_OPERATOR:-mad}"

# ============================================================
# Helper Functions
# ============================================================

log_info() {
    logger -t "$LOG_TAG" "[INFO] $1"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [INFO] $1"
}

log_warn() {
    logger -t "$LOG_TAG" "[WARN] $1"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [WARN] $1"
}

log_error() {
    logger -t "$LOG_TAG" "[ERROR] $1"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] [ERROR] $1"
}

# ============================================================
# Connection Functions
# ============================================================

check_tailscale_installed() {
    if ! command -v tailscale &> /dev/null; then
        log_error "Tailscale is not installed"
        return 1
    fi
    return 0
}

check_tailscale_service() {
    if ! systemctl is-active --quiet tailscaled; then
        log_warn "tailscaled service is not running"
        return 1
    fi
    return 0
}

check_tailscale_connected() {
    # Check if tailscale status returns successfully
    if ! tailscale status &> /dev/null; then
        return 1
    fi
    
    # Check if we have an IP address
    local ip=$(tailscale ip -4 2>/dev/null)
    if [[ -z "$ip" ]]; then
        return 1
    fi
    
    return 0
}

get_connection_status() {
    # Returns: connected, connecting, disconnected, needs_auth, error
    local status_output=$(tailscale status --json 2>/dev/null)
    
    if [[ -z "$status_output" ]]; then
        echo "error"
        return
    fi
    
    # Parse BackendState from JSON
    local backend_state=$(echo "$status_output" | grep -o '"BackendState":"[^"]*"' | cut -d'"' -f4)
    
    case "$backend_state" in
        "Running")
            echo "connected"
            ;;
        "Starting")
            echo "connecting"
            ;;
        "NeedsLogin"|"NeedsMachineAuth")
            echo "needs_auth"
            ;;
        "Stopped")
            echo "disconnected"
            ;;
        *)
            echo "error"
            ;;
    esac
}

restart_tailscale_service() {
    log_info "Restarting tailscaled service..."
    
    systemctl restart tailscaled
    sleep 5
    
    if systemctl is-active --quiet tailscaled; then
        log_info "tailscaled service restarted successfully"
        return 0
    else
        log_error "Failed to restart tailscaled service"
        return 1
    fi
}

reconnect_tailscale() {
    log_info "Attempting to reconnect Tailscale..."
    
    # Try to bring up Tailscale. We pass --accept-risk=lose-ssh because the
    # control link to the operator may itself ride Tailscale (LTE/Tailscale
    # link from GCS to Jetson), and --operator=$OPERATOR mirrors the value
    # set on the daemon so the partial-prefs check passes.
    tailscale up \
        --hostname="$HOSTNAME" \
        --operator="$OPERATOR" \
        --accept-risk=lose-ssh 2>&1 | while read -r line; do
        log_info "tailscale up: $line"
    done
    
    sleep 5
    
    if check_tailscale_connected; then
        local ip=$(tailscale ip -4 2>/dev/null)
        log_info "Tailscale reconnected successfully (IP: $ip)"
        return 0
    else
        log_warn "Tailscale reconnection attempt failed"
        return 1
    fi
}

attempt_recovery() {
    local retries=0
    
    while [[ $retries -lt $MAX_RETRIES ]]; do
        retries=$((retries + 1))
        log_info "Recovery attempt $retries of $MAX_RETRIES..."
        
        # First check if service is running
        if ! check_tailscale_service; then
            restart_tailscale_service
            sleep 5
        fi
        
        # Try to reconnect
        if reconnect_tailscale; then
            return 0
        fi
        
        if [[ $retries -lt $MAX_RETRIES ]]; then
            log_info "Waiting $RETRY_DELAY seconds before next attempt..."
            sleep $RETRY_DELAY
        fi
    done
    
    log_error "Failed to recover Tailscale connection after $MAX_RETRIES attempts"
    return 1
}

# ============================================================
# Main Monitoring Loop
# ============================================================

monitor_loop() {
    log_info "Starting Tailscale watchdog (check interval: ${CHECK_INTERVAL}s)"
    
    local consecutive_failures=0
    local last_status=""
    
    while true; do
        # Check if Tailscale is installed
        if ! check_tailscale_installed; then
            log_error "Tailscale not installed, exiting watchdog"
            exit 1
        fi
        
        # Get current status
        local current_status=$(get_connection_status)
        
        # Log status changes
        if [[ "$current_status" != "$last_status" ]]; then
            log_info "Status changed: $last_status -> $current_status"
            last_status="$current_status"
        fi
        
        case "$current_status" in
            "connected")
                consecutive_failures=0
                # All good, nothing to do
                ;;
            "connecting")
                # Wait for it to finish connecting
                log_info "Tailscale is connecting..."
                consecutive_failures=0
                ;;
            "needs_auth")
                log_warn "Tailscale needs authentication - manual intervention required"
                consecutive_failures=$((consecutive_failures + 1))
                ;;
            "disconnected"|"error")
                consecutive_failures=$((consecutive_failures + 1))
                log_warn "Tailscale disconnected (consecutive failures: $consecutive_failures)"
                
                if [[ $consecutive_failures -ge 2 ]]; then
                    log_info "Multiple failures detected, attempting recovery..."
                    if attempt_recovery; then
                        consecutive_failures=0
                    fi
                fi
                ;;
        esac
        
        sleep $CHECK_INTERVAL
    done
}

# ============================================================
# Signal Handling
# ============================================================

cleanup() {
    log_info "Watchdog shutting down..."
    exit 0
}

trap cleanup SIGTERM SIGINT

# ============================================================
# Entry Point
# ============================================================

main() {
    echo ""
    echo "=============================================="
    echo "    NOMAD Tailscale Connection Watchdog"
    echo "=============================================="
    echo ""
    
    # Initial status check
    if check_tailscale_installed; then
        log_info "Tailscale is installed"
        
        local ip=$(tailscale ip -4 2>/dev/null)
        if [[ -n "$ip" ]]; then
            log_info "Current IP: $ip"
        fi
        
        local status=$(get_connection_status)
        log_info "Current status: $status"
    else
        log_error "Tailscale is not installed"
        exit 1
    fi
    
    echo ""
    
    # Start monitoring
    monitor_loop
}

main "$@"
