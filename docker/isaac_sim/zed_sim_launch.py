# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""
NOMAD ZED Simulation Launch

Launches the ZED simulation publisher node that provides ZED-compatible
ROS2 topics from Isaac Sim. This is the sim equivalent of
`ros2 launch zed_wrapper zed_camera.launch.py` on the real Jetson.

Usage:
  ros2 launch nomad_zed_sim_launch zed_sim.launch.py

Or directly:
  python3 -m nomad_zed_sim_launch.zed_sim_launch
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name", default="zed")
    camera_model = LaunchConfiguration("camera_model", default="zed2i")
    grab_resolution = LaunchConfiguration("grab_resolution", default="HD720")
    depth_mode = LaunchConfiguration("depth_mode", default="NEURAL_LIGHT")
    sim_enabled = LaunchConfiguration("sim_enabled", default="true")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="zed"),
            DeclareLaunchArgument("camera_model", default_value="zed2i"),
            DeclareLaunchArgument("grab_resolution", default_value="HD720"),
            DeclareLaunchArgument("depth_mode", default_value="NEURAL_LIGHT"),
            DeclareLaunchArgument("sim_enabled", default_value="true"),
            ExecuteProcess(
                cmd=[
                    "python3",
                    "/opt/nomad/isaac_sim/zed_sim_publisher.py",
                ],
                output="screen",
                additional_env={
                    "ZED_CAMERA_NAME": camera_name,
                    "ZED_CAMERA_MODEL": camera_model,
                    "ZED_GRAB_RESOLUTION": grab_resolution,
                    "ZED_DEPTH_MODE": depth_mode,
                },
                condition=IfCondition(sim_enabled),
            ),
        ]
    )
