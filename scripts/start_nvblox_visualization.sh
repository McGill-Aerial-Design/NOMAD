#!/bin/bash
# NOMAD nvblox Visualization Startup Script
# Run this on the Jetson to start ZED camera + nvblox + RViz visualization
#
# Prerequisites:
# - Docker container 'nomad_isaac_ros_32' must be running
# - ZED camera connected via USB
# - X11 display available (DISPLAY=:1 with gdm on Ubuntu)
#
# Usage from Windows (PowerShell):
#   ssh mad@100.75.218.89 "bash /home/mad/NOMAD/scripts/start_nvblox_visualization.sh"

set -e

echo "=== NOMAD nvblox Visualization Startup ==="

# Configuration
CONTAINER="nomad_isaac_ros_32"
DISPLAY_NUM=":1"
XAUTHORITY_PATH="/run/user/1000/gdm/Xauthority"
ROS_SETUP="/workspaces/isaac_ros-dev/install/setup.bash"
RVIZ_CONFIG="/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/visualization/zed_example.rviz"
LOG_DIR="/tmp"

# Step 1: Check ZED camera is connected
echo "[1/5] Checking ZED camera connection..."
if ! lsusb | grep -qi "stereolabs"; then
    echo "ERROR: ZED camera not detected on USB!"
    echo "Please check the USB connection and try again."
    exit 1
fi
echo "ZED camera detected."

# Step 2: Check Docker container is running
echo "[2/5] Checking Docker container..."
if ! docker ps | grep -q "$CONTAINER"; then
    echo "ERROR: Docker container '$CONTAINER' is not running!"
    echo "Start it with: docker start $CONTAINER"
    exit 1
fi
echo "Container '$CONTAINER' is running."

# Step 3: Restart container to ensure fresh USB device access
echo "[3/5] Restarting container for fresh USB device binding..."
docker restart "$CONTAINER"
sleep 15
echo "Container restarted."

# Step 4: Enable X11 access for Docker
echo "[4/5] Enabling X11 display access..."
export DISPLAY="$DISPLAY_NUM"
export XAUTHORITY="$XAUTHORITY_PATH"
xhost +local:docker 2>/dev/null || echo "Note: xhost may already be configured"

# Step 5: Launch full ZED + nvblox stack
echo "[5/5] Launching ZED camera + nvblox + RViz..."
docker exec -d "$CONTAINER" /bin/bash -c "source $ROS_SETUP && ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2 > $LOG_DIR/zed_nvblox_full.log 2>&1"

echo "Waiting for ZED camera to initialize (30 seconds)..."
sleep 30

# Launch RViz
echo "Launching RViz visualization..."
docker exec -e DISPLAY="$DISPLAY_NUM" -e XAUTHORITY="$XAUTHORITY_PATH" -d "$CONTAINER" /bin/bash -c "source $ROS_SETUP && rviz2 -d $RVIZ_CONFIG > $LOG_DIR/rviz.log 2>&1"

sleep 5

# Verify everything is running
echo ""
echo "=== Verification ==="
echo "Checking ZED topics..."
docker exec "$CONTAINER" /bin/bash -c "source $ROS_SETUP && timeout 3 ros2 topic hz /zed/zed_node/left/image_rect_color --window 2" 2>&1 | head -3 || true

echo ""
echo "Checking nvblox mesh..."
docker exec "$CONTAINER" /bin/bash -c "source $ROS_SETUP && timeout 3 ros2 topic hz /nvblox_node/mesh --window 2" 2>&1 | head -3 || true

echo ""
echo "Checking RViz..."
docker exec "$CONTAINER" pgrep -la rviz || echo "WARNING: RViz may not be running"

echo ""
echo "=== Startup Complete ==="
echo "Check the Jetson display for RViz visualization."
echo "Logs available at:"
echo "  - $LOG_DIR/zed_nvblox_full.log"
echo "  - $LOG_DIR/rviz.log"
echo ""
echo "To stop: docker exec $CONTAINER pkill -f 'ros2|rviz'"
