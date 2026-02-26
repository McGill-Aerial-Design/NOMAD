#!/bin/bash
# =============================================================================
# NOMAD Restart Script - kills all services and restarts cleanly
# =============================================================================

echo "Stopping all NOMAD services..."

# Stop systemd service first (prevents Restart=always from respawning)
if systemctl is-active --quiet nomad 2>/dev/null; then
    echo "  Stopping systemd nomad.service..."
    sudo systemctl stop nomad 2>/dev/null || true
fi

# Kill all NOMAD processes
pkill -f "start_nomad_full.sh" 2>/dev/null || true
pkill -f "edge_core.main" 2>/dev/null || true
pkill -f "mavlink-routerd" 2>/dev/null || true
docker exec nomad_isaac_ros pkill -f "simple_video_bridge" 2>/dev/null || true
docker exec nomad_isaac_ros pkill -f "ros2 launch" 2>/dev/null || true
docker exec nomad_isaac_ros pkill -f "zed_node" 2>/dev/null || true
pkill -f "gst-launch" 2>/dev/null || true

# Wait and verify cleanup
sleep 2
if pgrep -f "edge_core.main" > /dev/null 2>&1; then
    echo "  Edge Core still alive, sending SIGKILL..."
    pkill -9 -f "edge_core.main" 2>/dev/null || true
    sleep 1
fi

echo "All NOMAD services stopped. Waiting 3 seconds before restart..."
sleep 3

# Start NOMAD services
cd ~/NOMAD
nohup bash scripts/run/start_nomad_full.sh all > /tmp/nomad_startup.log 2>&1 &
echo "NOMAD services starting in background. Check /tmp/nomad_startup.log for progress."
