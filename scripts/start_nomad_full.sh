#!/bin/bash
# ============================================================
# NOMAD Full System Startup Script
# ============================================================
# Starts Edge Core API and ZED Video Stream via MediaMTX
# 
# Stream URL: rtsp://<JETSON_IP>:8554/zed
# Multiple viewers supported (Mission Planner, VLC, phone, etc.)
# ============================================================

set -e

# Configuration
JETSON_IP=100.75.218.89
API_PORT=8000
RTSP_PORT=8554
LOG_DIR=/home/mad/nomad_logs
NOMAD_DIR=/home/mad/NOMAD

mkdir -p $LOG_DIR

echo "=========================================="
echo "  NOMAD System Startup (MediaMTX RTSP)"
echo "  AEAC 2026 - McGill Aerial Design"
echo "=========================================="
echo "Jetson IP:  $JETSON_IP"
echo "API Port:   $API_PORT"
echo "RTSP Port:  $RTSP_PORT"
echo ""

# Stop any existing services
echo "[1/5] Stopping existing services..."
pkill -9 -f "edge_core.main" 2>/dev/null || true
pkill -9 -f "gst-launch" 2>/dev/null || true
sleep 2

# Check if camera is still busy
echo "[2/5] Checking camera availability..."
if lsof /dev/video0 2>/dev/null | grep -v "^COMMAND"; then
    echo "    WARNING: Camera still in use"
    fuser -k /dev/video0 2>/dev/null || true
    sleep 2
fi

# Check MediaMTX is running
echo "[3/5] Checking MediaMTX RTSP server..."
if pgrep -f mediamtx > /dev/null; then
    echo "    OK: MediaMTX running"
else
    echo "    FAIL: MediaMTX not running! Start with: sudo systemctl start mediamtx"
    exit 1
fi

# Start Edge Core API
echo "[4/5] Starting Edge Core API..."
cd $NOMAD_DIR
export PATH=/home/mad/.local/bin:$PATH
export NOMAD_DEBUG=true
nohup python3 -m edge_core.main > $LOG_DIR/edge_core.log 2>&1 &
EDGE_PID=$!
sleep 2

# Verify Edge Core is running
if curl -s http://localhost:$API_PORT/health > /dev/null; then
    echo "    OK: Edge Core running (PID: $EDGE_PID)"
else
    echo "    FAIL: Edge Core failed to start!"
    cat $LOG_DIR/edge_core.log
    exit 1
fi

# Start ZED Video Stream -> MediaMTX
echo "[5/5] Starting ZED Video Stream (Ultra-Low Latency)..."
# Ultra-low latency H.264 pipeline:
# - Convert to I420 (YUV 4:2:0) for compatibility
# - x264enc: zerolatency tune, ultrafast preset, no B-frames
# - baseline profile: maximum compatibility
# - key-int-max=15: Frequent keyframes for fast recovery
gst-launch-1.0 -q \
  v4l2src device=/dev/video0 do-timestamp=true ! \
  "video/x-raw,width=2560,height=720,framerate=30/1" ! \
  videocrop left=0 right=1280 ! \
  videoconvert ! \
  "video/x-raw,format=I420" ! \
  x264enc tune=zerolatency bitrate=4000 speed-preset=ultrafast \
    sliced-threads=true key-int-max=15 bframes=0 \
    rc-lookahead=0 sync-lookahead=0 ! \
  h264parse ! \
  rtspclientsink location=rtsp://localhost:$RTSP_PORT/zed \
    latency=0 protocols=tcp > $LOG_DIR/video.log 2>&1 &
VIDEO_PID=$!

sleep 2

if ps -p $VIDEO_PID > /dev/null 2>&1; then
    echo "    OK: Video stream running (PID: $VIDEO_PID)"
else
    echo "    FAIL: Video stream failed to start!"
    echo "    Trying alternative UDP method..."
    cat $LOG_DIR/video.log
    
    # Fallback: Use ffmpeg if rtspclientsink fails
    if command -v ffmpeg &> /dev/null; then
        ffmpeg -f v4l2 -framerate 30 -video_size 2560x720 -i /dev/video0 \
            -filter:v "crop=1280:720:0:0" \
            -c:v libx264 -preset ultrafast -tune zerolatency -b:v 4000k \
            -f rtsp rtsp://localhost:$RTSP_PORT/zed > $LOG_DIR/video.log 2>&1 &
        VIDEO_PID=$!
        sleep 2
        
        if ps -p $VIDEO_PID > /dev/null 2>&1; then
            echo "    OK: Video stream (ffmpeg) running (PID: $VIDEO_PID)"
        else
            echo "    FAIL: ffmpeg also failed!"
            cat $LOG_DIR/video.log
        fi
    fi
fi

echo ""
echo "=========================================="
echo "  NOMAD System Running"
echo "=========================================="
echo "API:    http://$JETSON_IP:$API_PORT"
echo "Video:  rtsp://$JETSON_IP:$RTSP_PORT/zed"
echo "Logs:   $LOG_DIR/"
echo ""
echo "MediaMTX supports multiple viewers simultaneously!"
echo "Use in: Mission Planner, VLC, or any RTSP client"
echo ""
echo "Press Ctrl+C to stop all services"
echo ""

# Trap signals to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down NOMAD services..."
    kill $EDGE_PID 2>/dev/null || true
    kill $VIDEO_PID 2>/dev/null || true
    echo "Goodbye!"
}
trap cleanup EXIT INT TERM

# Wait for services to exit
wait
