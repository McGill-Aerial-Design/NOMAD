#!/bin/bash
# NOMAD nvblox Startup Script (Headless - No X11/RViz)
# Run this on the Jetson to start ZED camera + nvblox
#
# Prerequisites:
# - Docker container 'nomad_isaac_ros_32' must be running
# - ZED camera connected via USB
#
# Usage from Windows (PowerShell):
#   ssh mad@100.75.218.89 "bash /home/mad/NOMAD/scripts/start_nvblox_visualization.sh"

set -e

echo "=== NOMAD nvblox Startup (Headless) ==="

# Configuration
CONTAINER="nomad_isaac_ros_32"
ROS_SETUP="/workspaces/isaac_ros-dev/install/setup.bash"
LOG_DIR="/tmp"

# Step 1: Check ZED camera is connected
echo "[1/4] Checking ZED camera connection..."
if ! lsusb | grep -qi "stereolabs"; then
    echo "ERROR: ZED camera not detected on USB!"
    echo "Please check the USB connection and try again."
    exit 1
fi
echo "ZED camera detected."

# Step 2: Check Docker container is running
echo "[2/4] Checking Docker container..."
if ! docker ps | grep -q "$CONTAINER"; then
    echo "ERROR: Docker container '$CONTAINER' is not running!"
    echo "Start it with: docker start $CONTAINER"
    exit 1
fi
echo "Container '$CONTAINER' is running."

# Step 3: Restart container to ensure fresh USB device access
echo "[3/4] Restarting container for fresh USB device binding..."
docker restart "$CONTAINER"
sleep 15
echo "Container restarted."

# Step 4: Launch full ZED + nvblox stack (headless, no RViz)
echo "[4/4] Launching ZED camera + nvblox..."
docker exec -d "$CONTAINER" /bin/bash -c "source $ROS_SETUP && ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2 > $LOG_DIR/zed_nvblox_full.log 2>&1"

echo "Waiting for ZED camera to initialize (30 seconds)..."
sleep 30

# Verify everything is running
echo ""
echo "=== Verification ==="
echo "Checking ZED topics..."
docker exec "$CONTAINER" /bin/bash -c "source $ROS_SETUP && timeout 3 ros2 topic hz /zed/zed_node/left/image_rect_color --window 2" 2>&1 | head -3 || true

echo ""
echo "Checking nvblox mesh..."
docker exec "$CONTAINER" /bin/bash -c "source $ROS_SETUP && timeout 3 ros2 topic hz /nvblox_node/mesh --window 2" 2>&1 | head -3 || true

echo ""
echo "=== Startup Complete ==="
echo "nvblox running in headless mode."
echo "Mesh data available at: /nvblox_node/mesh"
echo "Log available at: $LOG_DIR/zed_nvblox_full.log"
echo ""
echo "To stop: docker exec $CONTAINER pkill -f ros2"
