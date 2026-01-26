#!/bin/bash
# Isaac ROS + ZED Setup Script
# Run this INSIDE the Isaac ROS Docker container after each restart

echo "=== Installing ROS2 Dependencies ==="
sudo apt update
sudo apt install -y \
  ros-humble-zed-msgs \
  ros-humble-robot-localization \
  ros-humble-point-cloud-transport \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools

echo ""
echo "=== Sourcing Workspace ==="
cd /workspaces/isaac_ros-dev
source install/setup.bash

echo ""
echo "=== Verifying ZED Camera ==="
if [ -e /dev/video0 ]; then
    echo "OK: ZED camera detected (/dev/video0)"
else
    echo "WARNING: ZED camera not detected!"
fi

echo ""
echo "=== Setup Complete ==="
echo ""
echo "To launch ZED + Nvblox:"
echo "  ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2"
echo ""
echo "To run NOMAD ROS-HTTP bridge (in another terminal):"
echo "  python3 /workspaces/isaac_ros-dev/src/ros_http_bridge.py --host 172.17.0.1 --port 8000"
