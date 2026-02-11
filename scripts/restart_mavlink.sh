#!/bin/bash
# =============================================================================
# Restart MAVLink Router
# =============================================================================
# Kills any running mavlink-routerd and restarts it with auto-discovered
# ground station IP (from Tailscale peers).  Called by the Edge Core
# COMMAND_WHITELIST ("restart_mavlink" / "start_mavlink").
#
# Exit codes:
#   0 - mavlink-routerd restarted successfully
#   1 - CubePilot not detected or restart failed
# =============================================================================

set -euo pipefail

LOG_DIR="${HOME}/nomad_logs"
mkdir -p "$LOG_DIR"

# Kill existing instance (ignore if not running)
pkill -f mavlink-routerd 2>/dev/null || true
sleep 1

# Verify CubePilot is connected
if [ ! -e /dev/ttyACM0 ]; then
    echo "no CubePilot detected at /dev/ttyACM0" >&2
    exit 1
fi

# Discover ground station IP from Tailscale peers
GCS_IP=$(tailscale status 2>/dev/null \
    | grep -v "$(hostname)" \
    | grep -oP '\d+\.\d+\.\d+\.\d+' \
    | head -1 || true)

if [ -z "$GCS_IP" ]; then
    GCS_IP="192.168.1.255"
    echo "warn: no Tailscale peer found, using fallback $GCS_IP" >&2
fi

# Start mavlink-routerd in background
nohup mavlink-routerd \
    -e "${GCS_IP}:14550" \
    -e 127.0.0.1:14550 \
    /dev/ttyACM0 \
    > "$LOG_DIR/mavlink.log" 2>&1 &

sleep 2

# Verify it started
if pgrep -f mavlink-routerd > /dev/null; then
    echo "mavlink-routerd restarted (GCS: ${GCS_IP})"
else
    echo "mavlink-routerd failed to start" >&2
    exit 1
fi
