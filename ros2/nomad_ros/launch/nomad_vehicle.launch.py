# SPDX-License-Identifier: Apache-2.0
"""Launch the NOMAD ROS 2 telemetry observer.

The adapter publishes validated MAVLink telemetry only. See the package README
for its observation topic contract and transitional data source.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory("nomad_ros")
    params_file = os.path.join(package_dir, "config", "params.yaml")

    node = Node(
        package="nomad_ros",
        executable="nomad_vehicle_node",
        name="nomad_vehicle_node",
        output="screen",
        parameters=[params_file],
    )
    return LaunchDescription([node])
