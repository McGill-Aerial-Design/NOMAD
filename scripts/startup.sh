#!/bin/bash
# NOMAD Jetson - Production Ready Startup Script
# Place in /home/mad/NOMAD/scripts/startup.sh

set -e

echo "================================"
echo "NOMAD Jetson Startup"
echo "================================"

# Load environment
source /etc/profile.d/nomad-env.sh 2>/dev/null || {
    echo "Setting up environment..."
    export PATH=/usr/local/cuda/bin:$PATH
    export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
    export CUDACXX=/usr/local/cuda/bin/nvcc
    export CUDA_HOME=/usr/local/cuda
    source /opt/ros/humble/setup.bash 2>/dev/null
    source /opt/ros/humble/install/setup.bash 2>/dev/null || true
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
}

# Check system health
echo "Checking system status..."
echo " - CPU Temp: $(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | head -1 | awk '{print $1/1000}')°C"
echo " - GPU: $(nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader || echo 'N/A')°C"
echo " - Disk: $(df -h / | awk 'NR==2 {print $5 " used"}')"

# Check Tailscale
if systemctl is-active --quiet tailscaled; then
    echo " - Tailscale: $(tailscale status --json | jq -r .BackendState 2>/dev/null || echo 'active')"
else
    echo " - Tailscale: STOPPED"
fi

# Check Edge Core
if systemctl is-active --quiet nomad; then
    echo " - Edge Core: RUNNING"
    curl -s http://localhost:8000/health > /dev/null && echo "   API: OK" || echo "   API: NOT RESPONDING"
else
    echo " - Edge Core: STOPPED"
    echo "   Starting Edge Core..."
    sudo systemctl start nomad
fi

# Check Docker
if systemctl is-active --quiet docker; then
    echo " - Docker: RUNNING"
    CONTAINERS=$(docker ps --format '{{.Names}}' | wc -l)
    echo "   Containers running: $CONTAINERS"
else
    echo " - Docker: STOPPED"
fi

# Check for hardware
echo ""
echo "Hardware Detection:"
if [ -d "/usr/local/zed" ]; then
    echo " - ZED SDK: INSTALLED"
    if lsusb | grep -qi "stereolabs"; then
        echo " - ZED Camera: CONNECTED"
    else
        echo " - ZED Camera: NOT DETECTED"
    fi
else
    echo " - ZED SDK: NOT INSTALLED"
fi

if [ -e "/dev/ttyACM0" ]; then
    echo " - Flight Controller: CONNECTED (/dev/ttyACM0)"
else
    echo " - Flight Controller: NOT DETECTED"
fi

# check ROS2
echo ""
echo "ROS2 Status:"
if command -v ros2 &> /dev/null; then
    echo " - ROS2: INSTALLED ($(ros2 doctor --report 2>&1 | grep 'ROS 2 version' || echo 'Humble'))"
else
    echo " - ROS2: NOT AVAILABLE"
fi

echo ""
echo "================================"
echo "System ready for operation"
echo "================================"
echo ""
echo "Useful commands:"
echo "  systemctl status nomad       - Check Edge Core"
echo "  docker ps                    - List containers"
echo "  journalctl -u nomad -f       - View Edge Core logs"
echo "  tailscale status             - Check VPN"
echo "  curl localhost:8000/health/detailed | jq  - Full system metrics"
echo ""
