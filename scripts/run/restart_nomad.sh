#!/bin/bash
# =============================================================================
# NOMAD Restart Script - kills all services and restarts cleanly
# =============================================================================

echo "Stopping all NOMAD services..."

# Stop systemd service first if we have sudo
if systemctl is-active --quiet nomad 2>/dev/null; then
    if sudo -n systemctl stop nomad 2>/dev/null; then
        echo "  Stopped systemd nomad.service"
        sleep 2
    else
        echo "  [WARN] Cannot stop systemd service (no sudo). Duplicates may occur."
    fi
fi

# Kill all NOMAD processes
pkill -f "start_nomad_full.sh" 2>/dev/null || true
pkill -f "edge_core.main" 2>/dev/null || true
pkill -f "mavlink-routerd" 2>/dev/null || true
# Kill ALL processes inside the Isaac ROS container (both launch paths)
docker exec nomad_isaac_ros pkill -f "launch_nvblox_bridge\.sh|launch_zed_nvblox\.sh" 2>/dev/null || true
docker exec nomad_isaac_ros pkill -f "simple_video_bridge" 2>/dev/null || true
docker exec nomad_isaac_ros pkill -f "ros_http_bridge" 2>/dev/null || true
docker exec nomad_isaac_ros pkill -f "ros2 launch" 2>/dev/null || true
docker exec nomad_isaac_ros pkill -f "component_container" 2>/dev/null || true
docker exec nomad_isaac_ros pkill -f "zed_node" 2>/dev/null || true
pkill -f "gst-launch" 2>/dev/null || true
# Clean up stale FastRTPS/DDS shared memory lock files inside container.
# pkill doesn't give ROS2 nodes time to release SHM ports, so leftover locks
# prevent the next launch from acquiring them (RTPS_TRANSPORT_SHM errors).
docker exec nomad_isaac_ros bash -c 'rm -f /dev/shm/fastrtps_* 2>/dev/null' || true

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
