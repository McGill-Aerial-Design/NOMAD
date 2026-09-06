# SPDX-License-Identifier: Apache-2.0
"""Launch the NOMAD ROS 2 vehicle adapter.

The adapter is a thin client of the NOMAD C++ core: it publishes typed
telemetry and translates standard ROS messages into core Vehicle calls. See
the package README for the topic and service contract.
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
