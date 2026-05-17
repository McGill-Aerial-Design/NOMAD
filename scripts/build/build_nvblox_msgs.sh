#!/bin/bash
# Build nvblox_msgs and dependencies inside the Isaac ROS container
set -e

source /opt/ros/humble/setup.bash 2>/dev/null
source /opt/ros/humble/install/setup.bash 2>/dev/null

# Add VPI to CMake search path
export CMAKE_PREFIX_PATH=/opt/nvidia/vpi3/lib/aarch64-linux-gnu/cmake:${CMAKE_PREFIX_PATH}

cd /workspaces/isaac_ros-dev

echo "=== Building nvblox_msgs (and dependencies) ==="
colcon build \
    --packages-up-to nvblox_msgs \
    --symlink-install \
    --cmake-args -Wno-dev \
    2>&1 | tail -30

echo "=== Verifying build ==="
source install/setup.bash 2>/dev/null
ros2 pkg list 2>/dev/null | grep -E 'nvblox|isaac'
echo "=== Build complete ==="
