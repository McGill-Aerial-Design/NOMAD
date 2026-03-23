# NOMAD ZED + nvblox + Nav2 + Custom Object Detection Launch File
#
# Wraps the standard zed_example.launch.py and adds:
# - camera=zed2 for the ZED 2i
# - Custom YOLO26 object detection via ZED SDK custom OD pipeline
# - Servo TF publisher (servo_mount -> camera_link dynamic transform)
# - Static TF: base_link -> servo_mount (mounting offset)
# - Nav2 stack with nvblox costmap for Jetson-side obstacle avoidance
# - Nav2 goal bridge (Edge Core API -> Nav2 actions)
#
# The ZED SDK loads the ONNX model, auto-converts to TensorRT on first run,
# and publishes 3D detections to /zed/zed_node/obj_det/objects.
#
# TF Tree (complete chain):
#   map -> odom -> base_link -> servo_mount -> camera_link
#                                              -> zed2i_left_camera_optical_frame
#
# Obstacle avoidance architecture (Jetson-side, ArduPlane has no avoidance):
#   nvblox (dynamic mapping) -> costmap (NvbloxCostmapLayer)
#   -> Nav2 (MPPI Omni controller + Smac2D planner) -> /cmd_vel
#   -> ros_http_bridge -> Edge Core -> NavController -> ArduPlane GUIDED
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
#   ros2 launch /workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py enable_od:=false
#   ros2 launch /workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py enable_nav2:=false

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nvblox_bringup_dir = get_package_share_directory('nvblox_examples_bringup')
    zed_example_launch = os.path.join(
        nvblox_bringup_dir, 'launch', 'zed_example.launch.py'
    )
    zed_launch = os.path.join(
        nvblox_bringup_dir, 'launch', 'sensors', 'zed.launch.py'
    )

    # Custom OD config path (ZED SDK YOLO26 circle detection)
    # This file configures the ZED node's custom_onnx_file and class parameters
    custom_od_config = '/workspaces/isaac_ros-dev/config/custom_circle_detection.yaml'

    # Nav2 config for omnidirectional drone
    nav2_params_file = '/workspaces/isaac_ros-dev/config/nav2_drone.yaml'

    enable_od_arg = DeclareLaunchArgument(
        'enable_od',
        default_value='true',
        description='Enable ZED custom object detection (YOLO26 circle detection)',
    )

    enable_nav2_arg = DeclareLaunchArgument(
        'enable_nav2',
        default_value='true',
        description='Enable Nav2 stack for Jetson-side obstacle avoidance',
    )

    # Servo TF publisher: publishes servo_mount -> camera_link at 50 Hz
    # reflecting the current servo pitch angle (TF-001)
    servo_tf_publisher = ExecuteProcess(
        cmd=[
            'python3',
            '/workspaces/isaac_ros-dev/edge_core/ros/servo_tf_publisher.py',
            '--host', '172.17.0.1',
            '--port', '8000',
            '--tf-rate', '50.0',
            '--poll-rate', '10.0',
        ],
        name='servo_tf_publisher',
        output='screen',
    )

    # NOTE: ZED SDK object detection params (od_enabled, custom_onnx_file, model, etc.)
    # are read at initialization time and are NOT dynamically reconfigurable.
    # The custom YOLO26 config is merged into zed_common.yaml BEFORE launch
    # by the api.py launch script, so ros2 param load is not needed here.

    # Nav2 stack: full navigation with nvblox costmap for obstacle avoidance
    # ArduPlane has no onboard obstacle avoidance - all avoidance is Jetson-side
    # MPPI Omni controller + Smac2D planner for omnidirectional copter mode
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_nav2')),
    )

    # Nav2 goal bridge: polls Edge Core for navigation goals, sends to Nav2
    # Bridges HTTP API commands to ROS2 Nav2 action servers
    nav2_goal_bridge = ExecuteProcess(
        cmd=[
            'python3',
            '/workspaces/isaac_ros-dev/edge_core/ros/nav2_goal_bridge.py',
            '--host', '172.17.0.1',
            '--port', '8000',
            '--rate', '2.0',
        ],
        name='nav2_goal_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_nav2')),
    )

    return LaunchDescription([
        enable_od_arg,
        enable_nav2_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_example_launch),
            launch_arguments={
                'camera': 'zed2',
            }.items(),
        ),
        servo_tf_publisher,
        nav2_launch,
        nav2_goal_bridge,
    ])
