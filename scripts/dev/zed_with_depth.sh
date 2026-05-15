#!/bin/bash
# Restart ZED + nvblox + bridge with depth explicitly enabled

docker exec nomad_isaac_ros bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null
  
  echo "Killing existing launch..."
  pkill -9 -f component_container_mt 2>/dev/null || true
  pkill -9 -f zed_node 2>/dev/null || true
  pkill -9 -f ros_http_bridge.py 2>/dev/null || true
  sleep 3
  
  echo "Starting fresh ZED + nvblox with DEPTH=true..."
  cd /workspaces/isaac_ros-dev
  
  # Use custom launch that forces depth to be published
  cat > /tmp/zed_nvblox_with_depth.launch.py << "LAUNCH"
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nvblox_bringup_dir = get_package_share_directory('"'"'nvblox_examples_bringup'"'"')
    zed_example_launch = os.path.join(
        nvblox_bringup_dir, '"'"'launch'"'"', '"'"'zed_example.launch.py'"'"'
    )
    return LaunchDescription([
        DeclareLaunchArgument('"'"'enable_od'"'"', default_value='"'"'false'"'"'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_example_launch),
            launch_arguments={
                '"'"'camera'"'"': '"'"'zed2'"'"',
                '"'"'od_enabled'"'"': LaunchConfiguration('"'"'enable_od'"'"'),
                '"'"'publish_depth'"'"': '"'"'true'"'"',
            }
        ),
    ])
LAUNCH
  
  ros2 launch /tmp/zed_nvblox_with_depth.launch.py &
  LAUNCH_PID=$!
  sleep 30
  
  echo "Starting bridge..."
  python3 /workspaces/isaac_ros-dev/edge_core/ros_http_bridge.py --host localhost --port 8000 --rate 30 --vio-topic /zed/zed_node/odom &
  BRIDGE_PID=$!
  
  echo "ZED+nvblox PIDs: $LAUNCH_PID, Bridge PID: $BRIDGE_PID"
  wait $LAUNCH_PID
'
