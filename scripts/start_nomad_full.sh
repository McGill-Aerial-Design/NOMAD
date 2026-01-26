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
# Stream URL: rtsp://<JETSON_IP>:8554/zed
# API URL: http://<JETSON_IP>:8000
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NOMAD_DIR="$(dirname "$SCRIPT_DIR")"

# Configuration
API_PORT=8000
RTSP_PORT=8554
LOG_DIR=/home/mad/nomad_logs
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
    
    nohup mavlink-routerd -e "$GCS_IP:14550" -e 127.0.0.1:14551 /dev/ttyACM0 > $LOG_DIR/mavlink.log 2>&1 &
    sleep 2
    
    if pgrep -f mavlink-routerd > /dev/null; then
        log_ok "MAVLink Router started (-> $GCS_IP:14550)"
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
        if [ -f "$NOMAD_DIR/infra/mediamtx.yml" ]; then
            nohup mediamtx "$NOMAD_DIR/infra/mediamtx.yml" > $LOG_DIR/mediamtx.log 2>&1 &
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
    pkill -f "edge_core.main" 2>/dev/null || true
    sleep 1
    
    cd $NOMAD_DIR
    export PATH=/home/mad/.local/bin:$PATH
    export NOMAD_DEBUG=true
    export NOMAD_LOG_DIR="$NOMAD_DIR/data/mission_logs"
    
    nohup python3 -m edge_core.main > $LOG_DIR/edge_core.log 2>&1 &
    EDGE_PID=$!
    sleep 3
    
    if curl -s http://localhost:$API_PORT/health > /dev/null; then
        log_ok "Edge Core running at http://localhost:$API_PORT (PID: $EDGE_PID)"
    else
        log_fail "Edge Core failed to start!"
        tail -20 $LOG_DIR/edge_core.log
    fi
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
# Start Multi-Stream Video
# -----------------------------------------------------------------------------

start_multi_stream() {
    log_info "Starting multi-stream video system..."
    
    if [ -f "$SCRIPT_DIR/start_multi_video.sh" ]; then
        bash "$SCRIPT_DIR/start_multi_video.sh" start
    else
        log_warn "Multi-stream video script not found"
    fi
}

# -----------------------------------------------------------------------------
# Start Video Stream (Task 1 Mode - direct GStreamer - DEPRECATED)
# -----------------------------------------------------------------------------

start_video_stream() {
    log_warn "Single-stream GStreamer mode is deprecated. Use multi-stream instead."
    # Kept for compatibility but not recommended
    log_info "Starting ZED Video Stream..."
    pkill -f "gst-launch" 2>/dev/null || true
    sleep 1

    # Try best resolution
    gst-launch-1.0 -q \
      v4l2src device=/dev/video0 do-timestamp=true ! \
      "video/x-raw,width=2560,height=720,framerate=30/1,format=YUY2" ! \
      videoconvert ! "video/x-raw,format=I420" ! \
      x264enc tune=zerolatency bitrate=6000 speed-preset=ultrafast \
        sliced-threads=true key-int-max=30 bframes=0 ! \
      h264parse ! \
      rtspclientsink location=rtsp://localhost:$RTSP_PORT/zed latency=0 protocols=tcp \
      > $LOG_DIR/video.log 2>&1 &
    VIDEO_PID=$!
    sleep 3
    
    if ps -p $VIDEO_PID > /dev/null 2>&1; then
        log_ok "Video stream running at 2560x720 (PID: $VIDEO_PID)"
    else
        log_warn "Video stream failed - trying lower resolution..."
        gst-launch-1.0 -q \
          v4l2src device=/dev/video0 do-timestamp=true ! \
          "video/x-raw,format=YUY2" ! \
          videoscale ! "video/x-raw,width=2560,height=720" ! \
          videoconvert ! "video/x-raw,format=I420" ! \
          x264enc tune=zerolatency bitrate=4000 speed-preset=ultrafast ! \
          h264parse ! \
          rtspclientsink location=rtsp://localhost:$RTSP_PORT/zed latency=0 protocols=tcp \
          > $LOG_DIR/video.log 2>&1 &
        VIDEO_PID=$!
        sleep 3
        
        if ps -p $VIDEO_PID > /dev/null 2>&1; then
            log_ok "Video stream running with auto-scaling"
        else
            log_fail "Video stream failed. Check $LOG_DIR/video.log"
        fi
    fi
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
    fi
    
    echo ""
    echo "Connection Info:"
    echo "  Tailscale IP: $JETSON_IP"
    echo "  API:          http://$JETSON_IP:$API_PORT"
    echo "  Video:        rtsp://$JETSON_IP:$RTSP_PORT/zed"
    echo "  MAVLink:      UDP $JETSON_IP:14550"
    echo ""
    echo "Logs:"
    echo "  MAVLink:    $LOG_DIR/mavlink.log"
    echo "  Edge Core:  $LOG_DIR/edge_core.log"
    echo "  Video:      $LOG_DIR/video.log"
    echo "  Isaac ROS:  docker logs nomad_isaac_ros"
    echo "=========================================="
}

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

main() {
    check_prerequisites
    
    case "$TASK_MODE" in
        task1)
            log_info "Starting Task 1 (GPS-based) services..."
            start_mavlink_router
            start_mediamtx
            start_edge_core
            start_isaac_ros
            start_multi_stream
            ;;
        task2)
            log_info "Starting Task 2 (VIO-based) services..."
            start_mavlink_router
            start_mediamtx
            start_edge_core
            start_isaac_ros
            start_multi_stream
            ;;
        all|*)
            log_info "Starting all services (Isaac ROS + Multi-Stream)..."
            start_mavlink_router
            start_mediamtx
            start_edge_core
            start_isaac_ros
            start_multi_stream
            ;;
    esac
    
    print_status
    
    log_ok "NOMAD startup complete!"
    echo ""
    echo "Press Ctrl+C to stop all services"
    echo ""
}

# Cleanup handler
cleanup() {
    echo ""
    log_info "Shutting down NOMAD services..."
    pkill -f "edge_core.main" 2>/dev/null || true
    pkill -f "gst-launch" 2>/dev/null || true
    # Don't stop Isaac ROS automatically
    echo "Goodbye!"
}
trap cleanup EXIT INT TERM

main "$@"

# Keep script running to allow Ctrl+C cleanup
wait 2>/dev/null || true
