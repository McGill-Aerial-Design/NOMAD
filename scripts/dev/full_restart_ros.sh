#!/bin/bash
# Full Isaac ROS restart: kill everything, start fresh together
CONTAINER_NAME="nomad_isaac_ros"

echo "=== Killing all ROS processes ==="
docker exec "$CONTAINER_NAME" bash -c "
  pkill -9 -f ros_http_bridge 2>/dev/null
  pkill -9 -f ros2 2>/dev/null
  pkill -9 -f component_container_mt 2>/dev/null
  pkill -9 -f zed_node 2>/dev/null
  sleep 2
  echo 'Killed old processes'
"

echo "=== Waiting for cleanup ==="
sleep 3

echo "=== Starting fresh ROS launch (ZED + nvblox + bridge) ==="
cat << 'LAUNCH_SH' > /tmp/full_launch.sh
#!/bin/bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/usr/local/zed/lib:$LD_LIBRARY_PATH

# Disable SHM transport to fix DDS port conflicts
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_no_shm.xml

# Write FastDDS profile that disables shared memory transport
cat > /tmp/fastdds_no_shm.xml << 'FASTDDS_XML'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>CustomUDPTransport</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="default_xrce_dds_participant" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>CustomUDPTransport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
FASTDDS_XML

echo "Starting ZED + nvblox launch..."
NOMAD_LAUNCH=/workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py
if [ -f "$NOMAD_LAUNCH" ]; then
    ros2 launch "$NOMAD_LAUNCH" &
else
  ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2 &
fi
NVBLOX_PID=$!
echo "nvblox launch PID: $NVBLOX_PID"

echo "Waiting 25s for ZED + nvblox..."
sleep 25

echo "Starting ROS-HTTP bridge..."
python3 /workspaces/isaac_ros-dev/edge_core/ros_http_bridge.py \
    --host localhost \
    --port 8000 \
    --rate 30 \
    --vio-topic /zed/zed_node/odom &
BRIDGE_PID=$!
echo "Bridge PID: $BRIDGE_PID"

wait $NVBLOX_PID
LAUNCH_SH

docker exec "$CONTAINER_NAME" chmod +x /tmp/full_launch.sh
docker exec -d "$CONTAINER_NAME" bash -c "nohup /tmp/full_launch.sh > /tmp/full_launch.log 2>&1"

echo "=== Launched! Waiting 30s ==="
sleep 30

echo "=== Launch log ==="
docker exec "$CONTAINER_NAME" tail -20 /tmp/full_launch.log 2>/dev/null

echo "=== Mesh status ==="
curl -s "http://localhost:8000/api/task/2/slam/mesh?format=summary" 2>/dev/null
