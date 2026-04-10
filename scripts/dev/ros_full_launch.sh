#!/bin/bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/usr/local/zed/lib:$LD_LIBRARY_PATH

# Disable SHM transport to fix DDS port conflicts when restarting processes
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
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_no_shm.xml

echo "Starting ZED + nvblox launch..."
NOMAD_LAUNCH=/workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py
if [ -f "$NOMAD_LAUNCH" ]; then
    ros2 launch "$NOMAD_LAUNCH" &
else
  ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2 &
fi
NVBLOX_PID=$!
echo "nvblox launch PID: $NVBLOX_PID"

echo "Waiting 25s for ZED + nvblox to start..."
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
