#!/bin/bash
# =============================================================================
# NOMAD Multi-Stream Video Manager
# =============================================================================
# Starts multiple video bridges for different ZED camera topics
# Each bridge publishes to a different MediaMTX stream
#
# Usage: ./start_multi_video.sh [start|stop|status]
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NOMAD_DIR="$(dirname "$SCRIPT_DIR")"
CONFIG_FILE="$NOMAD_DIR/config/video_streams.json"
LOG_DIR="/tmp/nomad_video"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_ok() { echo -e "${GREEN}[OK]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; }

mkdir -p "$LOG_DIR"

# =============================================================================
# Start Video Bridges
# =============================================================================

start_bridges() {
    log_info "Starting ROS video bridges..."
    
    # Start each video bridge from config
    # RGB Stream (main)
    log_info "Starting RGB stream bridge..."
    docker exec -d nomad_isaac_ros bash -c '
        source /opt/ros/humble/setup.bash && \
        source /workspaces/isaac_ros-dev/install/setup.bash && \
        python3 /tmp/ros_video_bridge.py \
            --topic /zed/zed_node/rgb/image_rect_color \
            --stream zed_rgb \
            --tcp-port 9999 \
            --host localhost \
            --port 8554 \
            --width 1280 \
            --height 720 \
            --fps 30 \
            > /tmp/video_bridge_rgb.log 2>&1
    '
    
    # Left camera
    log_info "Starting left camera stream bridge..."
    docker exec -d nomad_isaac_ros bash -c '
        source /opt/ros/humble/setup.bash && \
        source /workspaces/isaac_ros-dev/install/setup.bash && \
        python3 /tmp/ros_video_bridge.py \
            --topic /zed/zed_node/left/image_rect_color \
            --stream zed_left \
            --tcp-port 10000 \
            --host localhost \
            --port 8554 \
            --width 1280 \
            --height 720 \
            --fps 30 \
            > /tmp/video_bridge_left.log 2>&1
    '
    
    # Right camera
    log_info "Starting right camera stream bridge..."
    docker exec -d nomad_isaac_ros bash -c '
        source /opt/ros/humble/setup.bash && \
        source /workspaces/isaac_ros-dev/install/setup.bash && \
        python3 /tmp/ros_video_bridge.py \
            --topic /zed/zed_node/right/image_rect_color \
            --stream zed_right \
            --tcp-port 10001 \
            --host localhost \
            --port 8554 \
            --width 1280 \
            --height 720 \
            --fps 30 \
            > /tmp/video_bridge_right.log 2>&1
    '
    
    sleep 5
    log_ok "Video bridges started"
}

# =============================================================================
# Start FFmpeg Encoders
# =============================================================================

start_encoders() {
    log_info "Starting FFmpeg encoders..."
    
    # RGB encoder
    nohup ffmpeg -f rawvideo -pix_fmt bgr24 -s 1280x720 -r 30 \
        -i tcp://127.0.0.1:9999 \
        -c:v libx264 -preset ultrafast -tune zerolatency -crf 23 \
        -f rtsp -rtsp_transport tcp \
        rtsp://localhost:8554/zed_rgb \
        > "$LOG_DIR/ffmpeg_rgb.log" 2>&1 &
    
    # Left camera encoder  
    nohup ffmpeg -f rawvideo -pix_fmt bgr24 -s 1280x720 -r 30 \
        -i tcp://127.0.0.1:10000 \
        -c:v libx264 -preset ultrafast -tune zerolatency -crf 23 \
        -f rtsp -rtsp_transport tcp \
        rtsp://localhost:8554/zed_left \
        > "$LOG_DIR/ffmpeg_left.log" 2>&1 &
    
    # Right camera encoder
    nohup ffmpeg -f rawvideo -pix_fmt bgr24 -s 1280x720 -r 30 \
        -i tcp://127.0.0.1:10001 \
        -c:v libx264 -preset ultrafast -tune zerolatency -crf 23 \
        -f rtsp -rtsp_transport tcp \
        rtsp://localhost:8554/zed_right \
        > "$LOG_DIR/ffmpeg_right.log" 2>&1 &
    
    sleep 5
    log_ok "FFmpeg encoders started"
}

# =============================================================================
# Stop All Streams
# =============================================================================

stop_all() {
    log_info "Stopping video bridges and encoders..."
    
    # Stop FFmpeg encoders
    pkill -f "ffmpeg.*tcp://127.0.0.1:999" 2>/dev/null || true
    pkill -f "ffmpeg.*tcp://127.0.0.1:1000" 2>/dev/null || true
    
    # Stop video bridges (kill python ros_video_bridge processes)
    docker exec nomad_isaac_ros bash -c "pkill -f 'ros_video_bridge.py'" 2>/dev/null || true
    
    log_ok "All video streams stopped"
}

# =============================================================================
# Show Status
# =============================================================================

show_status() {
    echo ""
    echo "=========================================="
    echo "  NOMAD Video Streams Status"
    echo "=========================================="
    
    # Check MediaMTX streams
    if curl -s http://localhost:9997/v3/paths/list > /dev/null 2>&1; then
        echo ""
        echo "Available Streams:"
        curl -s http://localhost:9997/v3/paths/list | python3 -c "
import sys, json
try:
    data = json.load(sys.stdin)
    for item in data.get('items', []):
        name = item.get('name', '')
        ready = item.get('ready', False)
        tracks = item.get('tracks', [])
        status = '[LIVE]' if ready else '[--]'
        codec = tracks[0] if tracks else 'N/A'
        print(f'{status} {name:20} {codec}')
except Exception as e:
    print(f'Error: {e}')
" 2>/dev/null || echo "Failed to get stream status"
    else
        log_warn "MediaMTX API not accessible"
    fi
    
    echo ""
    echo "FFmpeg Encoders:"
    pgrep -af "ffmpeg.*tcp://127.0.0.1" | head -5 || echo "No encoders running"
    
    echo ""
    echo "=========================================="
}

# =============================================================================
# Main
# =============================================================================

case "${1:-start}" in
    start)
        log_info "Starting multi-stream video system..."
        start_bridges
        start_encoders
        show_status
        log_ok "Multi-stream video started"
        ;;
    stop)
        stop_all
        ;;
    status)
        show_status
        ;;
    restart)
        stop_all
        sleep 3
        start_bridges
        start_encoders
        show_status
        ;;
    *)
        echo "Usage: $0 {start|stop|status|restart}"
        exit 1
        ;;
esac
