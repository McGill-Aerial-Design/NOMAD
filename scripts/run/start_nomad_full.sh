#!/bin/bash
# =============================================================================
# NOMAD Full System Startup Script
# =============================================================================
# Starts all NOMAD services for competition:
#   - MAVLink Router (telemetry to Mission Planner)
#   - Edge Core API (REST API for all operations)
#   - MediaMTX RTSP (video streaming)
#   - Isaac ROS + ZED (Task 2 VIO) - optional
#
# Usage: ./start_nomad_full.sh [task1|task2|all]
#   task1 - Start only services needed for Task 1 (GPS-based)
#   task2 - Start services for Task 2 (VIO-based) including Isaac ROS
#   all   - Start everything (default)
#
# Stream URL: rtsp://<JETSON_IP>:8554/primary
# API URL: http://<JETSON_IP>:8000
# =============================================================================

# Do NOT use set -e -- individual service failures should not silently kill
# the entire startup. Each function handles its own errors and logs them.
# set -e  (intentionally disabled)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NOMAD_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Configuration
API_PORT=8000
RTSP_PORT=8554
# Use HOME environment variable for user-agnostic paths (default: /home/mad)
LOG_DIR=${HOME}/nomad_logs
TASK_MODE="${1:-all}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_ok() { echo -e "${GREEN}[OK]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; }

mkdir -p $LOG_DIR

# Cleanup old logs (keep last 48 hours or 500MB max)
log_info "Cleaning up logs older than 2 days..."
LOG_CLEANUP_AGE_DAYS=2
find ${LOG_DIR} -name "*.log" -mtime +${LOG_CLEANUP_AGE_DAYS} -delete 2>/dev/null || true
log_ok "Old logs cleaned"

# Get Tailscale IP
JETSON_IP=$(tailscale ip -4 2>/dev/null || hostname -I | awk '{print $1}')

echo "=========================================="
echo "  NOMAD System Startup"
echo "  Mode: $TASK_MODE"
echo "  AEAC 2026 - McGill Aerial Design"
echo "=========================================="
echo "Jetson IP:  $JETSON_IP"
echo "API Port:   $API_PORT"
echo "RTSP Port:  $RTSP_PORT"
echo ""

# -----------------------------------------------------------------------------
# Check Prerequisites
# -----------------------------------------------------------------------------

check_prerequisites() {
    log_info "Checking prerequisites..."
    
    if [ -e /dev/ttyACM0 ]; then
        log_ok "CubePilot detected at /dev/ttyACM0"
    else
        log_warn "CubePilot not detected - MAVLink will not connect"
    fi
    
    if lsusb | grep -qi "stereolabs"; then
        log_ok "ZED camera detected"
    else
        log_warn "ZED camera not detected"
    fi
    
    if tailscale status &>/dev/null; then
        log_ok "Tailscale connected: $JETSON_IP"
    else
        log_warn "Tailscale not connected"
    fi
}

# -----------------------------------------------------------------------------
# Start MAVLink Router
# -----------------------------------------------------------------------------

start_mavlink_router() {
    log_info "Starting MAVLink Router..."
    pkill -f mavlink-routerd 2>/dev/null || true
    sleep 1
    
    if [ ! -e /dev/ttyACM0 ]; then
        log_warn "Skipping MAVLink - no CubePilot connected"
        return
    fi
    
    # Get ground station IP
    GCS_IP=$(tailscale status 2>/dev/null | grep -v "$(hostname)" | grep -oP '\d+\.\d+\.\d+\.\d+' | head -1 || echo "")
    
    if [ -z "$GCS_IP" ]; then
        GCS_IP="192.168.1.255"
        log_warn "No Tailscale peer found, using: $GCS_IP"
    fi
    
    nohup mavlink-routerd -e "$GCS_IP:14550" -e 127.0.0.1:14550 /dev/ttyACM0 > $LOG_DIR/mavlink.log 2>&1 &
    sleep 2
    
    if pgrep -f mavlink-routerd > /dev/null; then
        log_ok "MAVLink Router started (-> $GCS_IP:14550 + local Edge Core)"
    else
        log_fail "MAVLink Router failed. Check $LOG_DIR/mavlink.log"
    fi
}

# -----------------------------------------------------------------------------
# Start MediaMTX
# -----------------------------------------------------------------------------

start_mediamtx() {
    log_info "Checking MediaMTX RTSP server..."
    if pgrep -f mediamtx > /dev/null; then
        log_ok "MediaMTX already running"
    else
        # Prefer systemd service (auto-restarts, boot-persistent)
        if systemctl --user is-enabled mediamtx &>/dev/null; then
            systemctl --user start mediamtx
            sleep 2
            if pgrep -f mediamtx > /dev/null; then
                log_ok "MediaMTX started via systemd on rtsp://localhost:$RTSP_PORT"
            else
                log_fail "MediaMTX systemd unit failed to start"
            fi
        elif [ -f "$NOMAD_DIR/infra/mediamtx.yml" ]; then
            MEDIAMTX_BIN=$(command -v mediamtx 2>/dev/null || echo "/home/mad/bin/mediamtx")
            nohup "$MEDIAMTX_BIN" "$NOMAD_DIR/infra/mediamtx.yml" > $LOG_DIR/mediamtx.log 2>&1 &
            sleep 2
            if pgrep -f mediamtx > /dev/null; then
                log_ok "MediaMTX started on rtsp://localhost:$RTSP_PORT"
            else
                log_fail "MediaMTX failed to start"
            fi
        else
            log_warn "MediaMTX config not found, skipping"
        fi
    fi
}

# -----------------------------------------------------------------------------
# Start Edge Core API
# -----------------------------------------------------------------------------

start_edge_core() {
    log_info "Starting Edge Core API..."
    
    # Stop systemd-managed instance first (Restart=always would respawn pkilled processes)
    # Use mask to prevent systemd from restarting the service during manual run
    if systemctl is-active --quiet nomad 2>/dev/null; then
        log_info "Masking and stopping systemd nomad.service to prevent duplicates..."
        sudo systemctl mask nomad 2>/dev/null || true
        sudo systemctl stop nomad 2>/dev/null || true
        sleep 2
    elif systemctl is-enabled --quiet nomad 2>/dev/null; then
        log_info "Masking systemd nomad.service to prevent auto-start..."
        sudo systemctl mask nomad 2>/dev/null || true
    fi
    
    # Kill any remaining edge_core processes (manual starts, orphans)
    pkill -f "edge_core.main" 2>/dev/null || true
    sleep 2
    
    # Verify no edge_core is still running
    if pgrep -f "edge_core.main" > /dev/null 2>&1; then
        log_warn "Edge Core still running after kill, sending SIGKILL..."
        pkill -9 -f "edge_core.main" 2>/dev/null || true
        sleep 1
    fi
    
    cd $NOMAD_DIR
    # Use HOME environment variable for user-agnostic Python local bin path
    export PATH=${HOME}/.local/bin:$PATH
    export NOMAD_DEBUG=true
    export NOMAD_LOG_DIR="$NOMAD_DIR/data/mission_logs"
    
    # Use venv Python if available (has pydantic, fastapi, etc.)
    PYTHON_BIN="python3"
    if [ -f "$NOMAD_DIR/venv/bin/python3" ]; then
        PYTHON_BIN="$NOMAD_DIR/venv/bin/python3"
        log_info "Using venv Python: $PYTHON_BIN"
    fi
    
    nohup $PYTHON_BIN -m edge_core.main > $LOG_DIR/edge_core.log 2>&1 &
    EDGE_PID=$!
    sleep 3
    
    # Wait for Edge Core to be ready (max 30 seconds)
    log_info "Waiting for Edge Core API to be ready..."
    for i in {1..30}; do
        if curl -s http://localhost:$API_PORT/health > /dev/null; then
            log_ok "Edge Core running at http://localhost:$API_PORT (PID: $EDGE_PID)"
            return 0
        fi
        sleep 1
    done
    
    log_fail "Edge Core failed to start!"
    tail -20 $LOG_DIR/edge_core.log
    return 1
}

# -----------------------------------------------------------------------------
# Start Isaac ROS (Task 2 Only)
# -----------------------------------------------------------------------------

start_isaac_ros() {
    log_info "Starting Isaac ROS + ZED..."
    
    if [ -f "$SCRIPT_DIR/start_isaac_ros_auto.sh" ]; then
        bash "$SCRIPT_DIR/start_isaac_ros_auto.sh" start
    else
        log_warn "Isaac ROS startup script not found"
    fi
}

# -----------------------------------------------------------------------------
# Start Video Bridge
# -----------------------------------------------------------------------------

start_video_bridge() {
    log_info "Starting video bridge..."
    
    # Wait for Edge Core API to be ready
    local max_wait=30
    local count=0
    log_info "Waiting for Edge Core API..."
    while [ $count -lt $max_wait ]; do
        if curl -s http://localhost:$API_PORT/health > /dev/null 2>&1; then
            log_ok "Edge Core API ready"
            break
        fi
        sleep 1
        count=$((count + 1))
    done
    
    if [ $count -eq $max_wait ]; then
        log_fail "Edge Core API not ready after ${max_wait}s, video bridge cannot start"
        echo "Check: curl http://localhost:$API_PORT/health" >> $LOG_DIR/video_bridge.log
        return 1
    fi
    
    # Wait for Isaac ROS container to be ready
    log_info "Waiting for Isaac ROS container..."
    count=0
    while [ $count -lt $max_wait ]; do
        if docker exec nomad_isaac_ros echo "ready" > /dev/null 2>&1; then
            log_ok "Isaac ROS container ready"
            break
        fi
        sleep 1
        count=$((count + 1))
    done
    
    if [ $count -eq $max_wait ]; then
        log_fail "Isaac ROS container not ready after ${max_wait}s, video bridge cannot start"
        echo "Container check failed at $(date)" >> $LOG_DIR/video_bridge.log
        docker ps -a >> $LOG_DIR/video_bridge.log 2>&1
        return 1
    fi
    
    # Give ZED camera time to initialize (increased from 5 to 10 seconds)
    log_info "Waiting for ZED camera initialization (10s)..."
    sleep 10
    
    # Start video bridge via API with retries
    log_info "Starting video bridge via API..."
    local max_retries=3
    local retry=0
    local success=false
    
    while [ $retry -lt $max_retries ]; do
        if [ $retry -gt 0 ]; then
            log_info "Retry $retry/$max_retries..."
            sleep 5
        fi
        
        local response=$(curl -s -w "\nHTTP_CODE:%{http_code}" -X POST http://localhost:$API_PORT/api/video/start 2>&1)
        local http_code=$(echo "$response" | grep "HTTP_CODE:" | cut -d: -f2)
        local body=$(echo "$response" | grep -v "HTTP_CODE:")
        
        # Log the attempt
        echo "[$(date)] Attempt $((retry+1)): HTTP $http_code - $body" >> $LOG_DIR/video_bridge.log
        
        if echo "$body" | grep -q '"success":true'; then
            log_ok "Video bridge started successfully"
            log_info "RTSP URL: rtsp://$JETSON_IP:$RTSP_PORT/primary"
            success=true
            break
        elif [ "$http_code" = "503" ]; then
            log_warn "Video stream manager not initialized yet (503), retrying..."
        elif [ "$http_code" = "500" ]; then
            log_warn "Video stream failed to start (500), retrying..."
        else
            log_warn "Unexpected response: HTTP $http_code - $body"
        fi
        
        retry=$((retry + 1))
    done
    
    if [ "$success" = false ]; then
        log_fail "Video bridge failed to start after $max_retries attempts"
        echo "FINAL FAILURE at $(date)" >> $LOG_DIR/video_bridge.log
        echo "Check edge_core logs: tail -50 $LOG_DIR/edge_core.log" | tee -a $LOG_DIR/video_bridge.log
        return 1
    fi
    
    return 0
}

# -----------------------------------------------------------------------------
# Print Status
# -----------------------------------------------------------------------------

print_status() {
    echo ""
    echo "=========================================="
    echo "  NOMAD System Running"
    echo "=========================================="
    
    # Get status from API
    if curl -s http://localhost:$API_PORT/health > /dev/null 2>&1; then
        curl -s http://localhost:$API_PORT/api/services/status 2>/dev/null | python3 -c "
import sys, json
try:
    data = json.load(sys.stdin)
    print('Service              Status')
    print('-' * 40)
    for svc, info in data.items():
        if isinstance(info, dict):
            stat = info.get('status', 'unknown')
            run = info.get('running', False)
            icon = '[OK]' if run else '[--]'
        else:
            stat = str(info)
            icon = '[--]'
        print(f'{icon} {svc:20} {stat}')
except: pass
" 2>/dev/null || true
        
        # Check video bridge status
        echo ""
        local video_status=$(curl -s http://localhost:$API_PORT/api/video/status 2>/dev/null)
        if echo "$video_status" | grep -q '"streaming":true'; then
            echo "[OK] Video Bridge       streaming"
        else
            echo "[--] Video Bridge       not streaming"
        fi
    fi
    
    echo ""
    echo "Connection Info:"
    echo "  Tailscale IP: $JETSON_IP"
    echo "  API:          http://$JETSON_IP:$API_PORT"
    echo "  Video:        rtsp://$JETSON_IP:$RTSP_PORT/primary"
    echo "  MAVLink:      UDP $JETSON_IP:14550"
    echo ""
    echo "Logs:"
    echo "  MAVLink:      $LOG_DIR/mavlink.log"
    echo "  Edge Core:    $LOG_DIR/edge_core.log"
    echo "  Video Bridge: $LOG_DIR/video_bridge.log"
    echo "  Isaac ROS:    docker logs nomad_isaac_ros"
    echo "=========================================="
}

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

main() {
    local failures=0
    check_prerequisites
    
    # NOTE: All task branches currently start the same set of services.
    # This is intentional -- future iterations will differentiate:
    #   task1: skip Isaac ROS / VIO (GPS-only)
    #   task2: skip MAVLink GPS forwarding, enable VIO-only mode
    # Keeping the branches separate now to make that refactor straightforward.
    
    log_info "Starting services for mode: $TASK_MODE"
    
    start_mavlink_router || { log_fail "MAVLink Router failed to start"; failures=$((failures + 1)); }
    start_mediamtx || { log_fail "MediaMTX failed to start"; failures=$((failures + 1)); }
    start_edge_core || { log_fail "Edge Core failed to start"; failures=$((failures + 1)); }
    start_isaac_ros || { log_warn "Isaac ROS failed to start (may not be needed)"; }
    start_video_bridge || { log_fail "Video Bridge failed to start"; failures=$((failures + 1)); }
    
    print_status
    
    if [ $failures -gt 0 ]; then
        log_warn "NOMAD startup completed with $failures service failure(s)"
        echo "Check logs in $LOG_DIR for details"
    else
        log_ok "NOMAD startup complete!"
    fi
    echo ""
    echo "Press Ctrl+C to stop all services"
    echo ""
}

# Cleanup handler
cleanup() {
    echo ""
    log_info "Shutting down NOMAD services..."
    
    # Stop Edge Core API
    pkill -f "edge_core.main" 2>/dev/null || true
    
    # Stop video bridge inside Docker container
    docker exec nomad_isaac_ros pkill -f "simple_video_bridge" 2>/dev/null || true
    
    # Stop any GStreamer processes
    pkill -f "gst-launch" 2>/dev/null || true
    
    # Stop MAVLink router
    pkill -f "mavlink-routerd" 2>/dev/null || true
    
    # Unmask systemd service so it can auto-start on next boot
    sudo systemctl unmask nomad 2>/dev/null || true
    
    log_ok "All services stopped"
    echo "Goodbye!"
}
trap cleanup EXIT INT TERM

main "$@"

# Keep script running to allow Ctrl+C cleanup
wait 2>/dev/null || true
