#!/bin/bash
# ============================================================
# NOMAD Full System Startup Script
# ============================================================
# Starts Edge Core API and ZED RTSP Video Server
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
echo "  NOMAD System Startup (RTSP Mode)"
echo "  AEAC 2026 - McGill Aerial Design"
echo "=========================================="
echo "Jetson IP:  $JETSON_IP"
echo "API Port:   $API_PORT"
echo "RTSP Port:  $RTSP_PORT"
echo ""

# Stop any existing services
echo "[1/4] Stopping existing services..."
pkill -9 -f "edge_core.main" 2>/dev/null || true
pkill -9 -f "edge_core.rtsp_server" 2>/dev/null || true
pkill -9 -f "gst-launch" 2>/dev/null || true
sleep 2

# Check if camera is still busy
echo "[2/4] Checking camera availability..."
if lsof /dev/video0 2>/dev/null | grep -v "^COMMAND"; then
    echo "    WARNING: Camera still in use"
    fuser -k /dev/video0 2>/dev/null || true
    sleep 2
fi

# Start Edge Core API
echo "[3/4] Starting Edge Core API..."
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

# Start ZED RTSP Video Server
echo "[4/4] Starting ZED RTSP Server..."
cd $NOMAD_DIR
nohup python3 -m edge_core.rtsp_server --port $RTSP_PORT > $LOG_DIR/rtsp_server.log 2>&1 &
RTSP_PID=$!
sleep 2

if ps -p $RTSP_PID > /dev/null 2>&1; then
    echo "    OK: RTSP server running (PID: $RTSP_PID)"
else
    echo "    FAIL: RTSP server failed to start!"
    cat $LOG_DIR/rtsp_server.log
    exit 1
fi

echo ""
echo "=========================================="
echo "  NOMAD System Running"
echo "=========================================="
echo "API:    http://$JETSON_IP:$API_PORT"
echo "Video:  rtsp://$JETSON_IP:$RTSP_PORT/zed"
echo "Logs:   $LOG_DIR/"
echo ""
echo "RTSP supports multiple viewers simultaneously!"
echo "Use in: Mission Planner, VLC, or any RTSP client"
echo ""
echo "Press Ctrl+C to stop all services"
echo ""

# Trap signals to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down NOMAD services..."
    kill $EDGE_PID 2>/dev/null || true
    kill $RTSP_PID 2>/dev/null || true
    echo "Goodbye!"
}
trap cleanup EXIT INT TERM

# Wait for services to exit
wait
