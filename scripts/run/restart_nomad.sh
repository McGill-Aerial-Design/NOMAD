#!/bin/bash
# Kill all NOMAD processes
pkill -f "start_nomad_full.sh" 2>/dev/null || true
pkill -f "edge_core.main" 2>/dev/null || true
pkill -f "mavlink-routerd" 2>/dev/null || true
docker exec nomad_isaac_ros pkill -f "simple_video_bridge" 2>/dev/null || true
docker exec nomad_isaac_ros pkill -f "ros2 launch" 2>/dev/null || true
docker exec nomad_isaac_ros pkill -f "zed_node" 2>/dev/null || true
pkill -f "gst-launch" 2>/dev/null || true

echo "All NOMAD services stopped. Waiting 3 seconds before restart..."
sleep 3

# Start NOMAD services
cd ~/NOMAD
nohup bash scripts/run/start_nomad_full.sh all > /tmp/nomad_startup.log 2>&1 &
echo "NOMAD services starting in background. Check /tmp/nomad_startup.log for progress."
