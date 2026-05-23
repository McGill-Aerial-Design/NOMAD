"""
Launch file for the target localizer node.

Usage:
  # Default: load competition.yaml from the package share dir.
  ros2 launch target_localizer target_localizer.launch.py

  # Override the params file path:
  ros2 launch target_localizer target_localizer.launch.py \
      params_file:=/abs/path/to/your_competition.yaml

  # Quick rectangle override (CLI args go through the params file in
  # competition.yaml -- edit that file or pass a custom one for the real
  # building polygon).

The building polygon now lives on the ground station — this node only
publishes targets as absolute (lat, lon, height_AGL). See competition.yaml
for the (much shorter) operational checklist.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("target_localizer"),
        "config",
        "competition.yaml",
    )

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the target_localizer params YAML",
        ),
    ]

    target_localizer_node = Node(
        package="target_localizer",
        executable="target_localizer_node",
        name="target_localizer",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[
            # NOMAD launches ZED with camera:=zed2 and SDK 5.2 topic naming.
            ("/zed2i/zed_node/rgb/image_rect_color",
             "/zed/zed_node/rgb/color/rect/image"),
            ("/zed2i/zed_node/depth/depth_registered",
             "/zed/zed_node/depth/depth_registered"),
            ("/zed2i/zed_node/rgb/camera_info",
             "/zed/zed_node/rgb/color/rect/camera_info"),
        ],
    )

    return LaunchDescription(args + [target_localizer_node])
