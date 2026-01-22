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

# Start MAVLink Router
echo "[3.5/5] Checking MAVLink Router..."
if pgrep -x mavlink-routerd > /dev/null; then
    echo "    OK: MAVLink Router running"
else
    echo "    Starting MAVLink Router..."
    # Check if binary exists
    if command -v mavlink-routerd &> /dev/null; then
        nohup mavlink-routerd -c $NOMAD_DIR/transport/mavlink_router/mavlink-router.conf > $LOG_DIR/mavlink.log 2>&1 &
        sleep 2
    else
        echo "    WARNING: mavlink-routerd not found! Systems may not connect to Flight Controller."
    fi
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
echo "[5/5] Starting ZED Video Stream (Best Available Resolution)..."
# Try resolutions in priority order: 2560x720 (stereo), 1344x376 (degraded), auto

# Method 1: Try full stereo resolution 2560x720 @ 30fps
echo "    Attempting 2560x720 @ 30fps..."
gst-launch-1.0 -q \
  v4l2src device=/dev/video0 do-timestamp=true ! \
  "video/x-raw,width=2560,height=720,framerate=30/1,format=YUY2" ! \
  videoconvert ! \
  "video/x-raw,format=I420" ! \
  x264enc tune=zerolatency bitrate=6000 speed-preset=ultrafast \
    sliced-threads=true key-int-max=30 bframes=0 \
    rc-lookahead=0 sync-lookahead=0 ! \
  h264parse ! \
  rtspclientsink location=rtsp://localhost:$RTSP_PORT/zed \
    latency=0 protocols=tcp > $LOG_DIR/video.log 2>&1 &
VIDEO_PID=$!
sleep 3

if ps -p $VIDEO_PID > /dev/null 2>&1; then
    echo "    OK: Video stream running at 2560x720 (PID: $VIDEO_PID)"
    VIDEO_RESOLUTION="2560x720"
else
    echo "    Failed: 2560x720 not supported"
    cat $LOG_DIR/video.log
    
    # Method 2: Try degraded resolution 1344x376 @ 15fps and scale up
    echo "    Attempting 1344x376 @ 15fps with upscaling..."
    gst-launch-1.0 -q \
      v4l2src device=/dev/video0 do-timestamp=true ! \
      "video/x-raw,width=1344,height=376,framerate=15/1,format=YUY2" ! \
      videoscale ! \
      "video/x-raw,width=2560,height=720" ! \
      videoconvert ! \
      "video/x-raw,format=I420" ! \
      x264enc tune=zerolatency bitrate=4000 speed-preset=ultrafast \
        sliced-threads=true key-int-max=30 bframes=0 ! \
      h264parse ! \
      rtspclientsink location=rtsp://localhost:$RTSP_PORT/zed \
        latency=0 protocols=tcp > $LOG_DIR/video.log 2>&1 &
    VIDEO_PID=$!
    sleep 3
    
    if ps -p $VIDEO_PID > /dev/null 2>&1; then
        echo "    OK: Video stream running at 1344x376 upscaled (PID: $VIDEO_PID)"
        VIDEO_RESOLUTION="1344x376 (upscaled to 2560x720)"
    else
        echo "    Failed: 1344x376 also failed"
        cat $LOG_DIR/video.log
        
        # Method 3: Auto-negotiate and scale to standard resolution
        echo "    Attempting auto-negotiation with scaling..."
        gst-launch-1.0 -q \
          v4l2src device=/dev/video0 do-timestamp=true ! \
          "video/x-raw,format=YUY2" ! \
          videoscale ! \
          "video/x-raw,width=2560,height=720" ! \
          videoconvert ! \
          "video/x-raw,format=I420" ! \
          x264enc tune=zerolatency bitrate=4000 speed-preset=ultrafast \
            sliced-threads=true key-int-max=30 bframes=0 ! \
          h264parse ! \
          rtspclientsink location=rtsp://localhost:$RTSP_PORT/zed \
            latency=0 protocols=tcp > $LOG_DIR/video.log 2>&1 &
        VIDEO_PID=$!
        sleep 3
        
        if ps -p $VIDEO_PID > /dev/null 2>&1; then
            echo "    OK: Video stream running with auto-resolution (PID: $VIDEO_PID)"
            VIDEO_RESOLUTION="auto (scaled to 2560x720)"
        else
            echo "    FAIL: All methods failed! Check camera"
            cat $LOG_DIR/video.log
            VIDEO_RESOLUTION="Failed"
        fi
    fi
fi

echo ""
echo "=========================================="
echo "  NOMAD System Running"
echo "=========================================="
echo "API:    http://$JETSON_IP:$API_PORT"
echo "Video:  rtsp://$JETSON_IP:$RTSP_PORT/zed (2560x720 stereo)"
echo "Logs:   $LOG_DIR/"
echo ""
echo "Stereo stream: 2560x720 side-by-side (Left + Right)"
echo "Mission Planner crops to left camera (1280x720) in HUD"
echo "MediaMTX supports multiple viewers simultaneously!"
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
