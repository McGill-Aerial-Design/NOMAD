#!/bin/bash
# Restart nvblox launch inside the Isaac ROS container
# The previous launch crashed because Edge Core was holding the camera.
# Now that Edge Core has been restarted (and the calibration camera leak fixed),
# the camera should be free.

CONTAINER_NAME="nomad_isaac_ros"

echo "=== Stopping old processes ==="
docker exec $CONTAINER_NAME bash -c '
  # Kill any zombie component_container_mt or launch processes
  pkill -9 -f component_container_mt 2>/dev/null || true
  pkill -9 -f "ros2 launch" 2>/dev/null || true
  pkill -9 -f launch_nvblox_bridge 2>/dev/null || true
  sleep 2
  echo "Cleaned up old processes"
'

echo "=== Writing fresh launch script ==="
docker exec -i $CONTAINER_NAME tee /tmp/launch_nvblox_bridge.sh > /dev/null << 'LAUNCH_SCRIPT'
#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/usr/local/zed/lib:$LD_LIBRARY_PATH

# Keep ZED at 360p (pub_downscale_factor: 2.0) to prevent cudaErrorIllegalAddress
# on 8GB Jetson Orin Nano when nvblox allocates GPU memory

# Overlay NOMAD nvblox config
NOMAD_CFG=/workspaces/isaac_ros-dev/config/nvblox_performance.yaml
NVBLOX_BASE=$(python3 -c "from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory('nvblox_examples_bringup'))" 2>/dev/null)/config/nvblox/nvblox_base.yaml
if [ -f "$NOMAD_CFG" ] && [ -f "$NVBLOX_BASE" ]; then
    echo "Applying NOMAD nvblox config"
    cp "$NOMAD_CFG" "$NVBLOX_BASE"
fi

# Use custom NOMAD launch with OD disabled by default
NOMAD_LAUNCH=/workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py
if [ -f "$NOMAD_LAUNCH" ]; then
    echo "Launching with custom launch file (ZED OD disabled)"
    ros2 launch "$NOMAD_LAUNCH" enable_od:=false
else
    echo "Custom launch not found, falling back to stock"
    ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2 enable_od:=false
fi
LAUNCH_SCRIPT
docker exec $CONTAINER_NAME chmod +x /tmp/launch_nvblox_bridge.sh

echo "=== Launching nvblox + ZED ==="
docker exec -d $CONTAINER_NAME bash -c \
    "bash /tmp/launch_nvblox_bridge.sh > /tmp/zed_nvblox.log 2>&1 & echo \$! > /tmp/zed_nvblox.pid"

echo "=== Waiting 20s for ZED to initialize ==="
sleep 20

echo "=== ZED nvblox log (last 15 lines) ==="
docker exec $CONTAINER_NAME tail -15 /tmp/zed_nvblox.log 2>/dev/null

echo ""
echo "=== Restarting ROS-HTTP bridge ==="
docker exec $CONTAINER_NAME bash -c 'pkill -f ros_http_bridge.py 2>/dev/null; sleep 2'
docker exec -i $CONTAINER_NAME tee /tmp/launch_bridge.sh > /dev/null << 'BRIDGE_SCRIPT'
#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
sleep 3
python3 /workspaces/isaac_ros-dev/edge_core/ros_http_bridge.py --host localhost --port 8000 --rate 30 --vio-topic /zed/zed_node/odom
BRIDGE_SCRIPT
docker exec $CONTAINER_NAME chmod +x /tmp/launch_bridge.sh
docker exec -d $CONTAINER_NAME bash -c \
    "nohup /tmp/launch_bridge.sh > /tmp/ros_bridge.log 2>&1 & echo \$! > /tmp/ros_bridge.pid"

echo "=== Done. Check mesh topics in ~10s ==="
