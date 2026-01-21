#!/bin/bash
# NOMAD Full System Startup Script
# Starts both Edge Core API and ZED Video Stream

set -e

GCS_IP=100.76.127.17
API_PORT=8000
VIDEO_PORT=5600
LOG_DIR=/home/mad/nomad_logs
mkdir -p $LOG_DIR

echo "=========================================="
echo "  NOMAD System Startup"
echo "=========================================="
echo "Ground Station: $GCS_IP"
echo "API Port: $API_PORT"
echo "Video Port: $VIDEO_PORT"
echo ""

# Stop any existing services
echo "[1/3] Stopping existing services..."
pkill -f "edge_core.main" 2>/dev/null || true
pkill -f "gst-launch" 2>/dev/null || true
sleep 1

# Start Edge Core API
echo "[2/3] Starting Edge Core API..."
cd /home/mad/NOMAD
export PATH=/home/mad/.local/bin:$PATH
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

# Start ZED Video Stream
echo "[3/3] Starting ZED Video Stream..."
gst-launch-1.0 -q \
  v4l2src device=/dev/video0 ! \
  "video/x-raw,width=2560,height=720,framerate=30/1" ! \
  videocrop left=0 right=1280 ! \
  videoconvert ! \
  x264enc tune=zerolatency bitrate=3000 speed-preset=superfast key-int-max=30 ! \
  "video/x-h264,stream-format=byte-stream" ! \
  rtph264pay config-interval=1 pt=96 mtu=1400 ! \
  udpsink host=$GCS_IP port=$VIDEO_PORT > $LOG_DIR/video.log 2>&1 &
VIDEO_PID=$!
sleep 1

if ps -p $VIDEO_PID > /dev/null 2>&1; then
    echo "    OK: Video stream running (PID: $VIDEO_PID)"
else
    echo "    FAIL: Video stream failed to start!"
fi

echo ""
echo "=========================================="
echo "  NOMAD System Running"
echo "=========================================="
echo "API:   http://100.75.218.89:$API_PORT"
echo "Video: udp://$GCS_IP:$VIDEO_PORT"
echo "Logs:  $LOG_DIR/"
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
