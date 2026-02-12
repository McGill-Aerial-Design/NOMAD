# NOMAD ZED + nvblox Launch File
#
# Simple wrapper around the standard zed_example.launch.py that sets
# camera=zed2 for the ZED 2i.
#
# NOTE: nvblox parameter overrides (voxel_size, ESDF mode, rates, etc.)
# are applied by the startup script (start_isaac_ros_auto.sh) which copies
# config/nvblox_performance.yaml over the installed nvblox_base.yaml
# BEFORE this launch file runs. This is necessary because nvblox loads
# parameters from YAML config files into a ComposableNode, and
# SetParameter in the launch description does not override those.
#
# Usage (inside Isaac ROS container):
#   ros2 launch /workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nvblox_bringup_dir = get_package_share_directory('nvblox_examples_bringup')
    zed_example_launch = os.path.join(
        nvblox_bringup_dir, 'launch', 'zed_example.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_example_launch),
            launch_arguments={'camera': 'zed2'}.items(),
        ),
    ])
