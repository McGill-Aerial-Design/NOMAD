# NOMAD ZED + nvblox + Nav2 + Custom Object Detection Launch File
#
# ZED and nvblox run in SEPARATE composable containers so a nvblox SIGABRT
# (CUDA memory error) does not kill the ZED node. Video, VIO, and telemetry
# survive nvblox crashes.
#
# Adds:
# - camera=zed2 for the ZED 2 (with correct config stack)
# - Servo TF publisher (servo_mount -> camera_link dynamic transform)
# - Static TF alias: zed_left_camera_frame_optical -> zed_left_camera_optical_frame
# - Obstacle distance bridge (nvblox ESDF -> MAVLink OBSTACLE_DISTANCE)
# - Nav2 stack with nvblox costmap for Jetson-side obstacle avoidance (optional)
# - Nav2 goal bridge (Edge Core API -> Nav2 actions) (optional)
#
# TF Tree (complete chain):
#   map -> odom -> zed_camera_link -> zed_camera_center -> zed_left_camera_frame
#                                                          -> zed_left_camera_frame_optical
#                                                             -> zed_left_camera_optical_frame (alias)
#
# NOTE: ZED config patches (publish_left_right, pub_downscale_factor, publish_mag, etc.)
# are applied by start_isaac_ros_auto.sh BEFORE this launch file runs because ZED
# reads them at initialization time. They affect common_stereo.yaml on disk.
#
# NOTE: nvblox parameter overrides (voxel_size, ESDF mode, rates, etc.)
# are applied by the startup script which copies config/nvblox_performance.yaml
# over the installed nvblox_base.yaml BEFORE this launch file runs.
#
# Usage (inside Isaac ROS container):
#   ros2 launch /workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py
#   ros2 launch .../nomad_zed_nvblox.launch.py enable_nvblox:=false
#   ros2 launch .../nomad_zed_nvblox.launch.py enable_nav2:=true

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nvblox_bringup_dir = get_package_share_directory("nvblox_examples_bringup")
    zed_wrapper_dir = get_package_share_directory("zed_wrapper")

    # ZED-only launch (no nvblox) — used when enable_nvblox:=false.
    zed_only_launch = os.path.join(
        nvblox_bringup_dir, "launch", "sensors", "zed.launch.py"
    )

    # Config files for ZED node (loaded in priority order, later overrides earlier).
    # common_stereo.yaml is patched by start_isaac_ros_auto.sh before launch:
    #   - publish_left_right: true  (lazy-published; only streams when subscribed)
    #   - publish_raw: true
    #   - pub_downscale_factor: 2.0 (360p — 720p causes CUDA OOM with nvblox on 8GB Orin Nano)
    zed_stereo_config = os.path.join(
        zed_wrapper_dir, "config", "common_stereo.yaml"
    )
    # nvblox bringup's ZED overrides (sensors.publish_mag etc.)
    nvblox_zed_common_config = os.path.join(
        nvblox_bringup_dir, "config", "sensors", "zed_common.yaml"
    )
    # Camera-model-specific params (grab_resolution, min_depth, max_depth for ZED 2)
    nvblox_zed2_config = os.path.join(
        nvblox_bringup_dir, "config", "sensors", "zed2.yaml"
    )

    # nvblox config files (copied/patched by start_isaac_ros_auto.sh before launch)
    nvblox_base_config = os.path.join(
        nvblox_bringup_dir, "config", "nvblox", "nvblox_base.yaml"
    )
    nvblox_zed_config = os.path.join(
        nvblox_bringup_dir, "config", "nvblox", "specializations", "nvblox_zed.yaml"
    )

    xacro_path = os.path.join(zed_wrapper_dir, "urdf", "zed_descr.urdf.xacro")

    # Nav2 config for omnidirectional drone
    nav2_params_file = "/workspaces/isaac_ros-dev/config/nav2_drone.yaml"

    enable_od_arg = DeclareLaunchArgument(
        "enable_od",
        default_value="false",
        description="Enable ZED custom object detection (YOLO26 circle detection) - DISABLED by default to prevent VRAM exhaustion and composable node instability on 8GB Jetson Orin Nano",
    )

    enable_nav2_arg = DeclareLaunchArgument(
        "enable_nav2",
        default_value="false",
        description="Enable Nav2 stack for Jetson-side obstacle avoidance with nvblox costmap (disabled by default to prevent duplicate instances from repeated launches)",
    )

    enable_foxglove_arg = DeclareLaunchArgument(
        "enable_foxglove",
        default_value="false",
        description="Enable Foxglove bridge for ROS2 topic visualization in Foxglove Studio (WebSocket on port 8765)",
    )

    enable_nvblox_arg = DeclareLaunchArgument(
        "enable_nvblox",
        default_value="true",
        description="Launch ZED + nvblox in SEPARATE containers (true) or ZED standalone only (false). Separate containers mean nvblox SIGABRT does not kill ZED.",
    )

    # Servo TF publisher: publishes servo_mount -> camera_link at 20 Hz
    # reflecting the current servo pitch angle, and publishes odom -> base_link
    # by inverting the camera odom pose so the full TF chain is connected.
    servo_tf_publisher = ExecuteProcess(
        cmd=[
            "python3",
            "/workspaces/isaac_ros-dev/edge_core/ros/servo_tf_publisher.py",
            "--host",
            "172.17.0.1",
            "--port",
            "8000",
            "--tf-rate",
            "20.0",
            "--poll-rate",
            "10.0",
            "--odom-topic",
            "/zed/zed_node/odom",
        ],
        name="servo_tf_publisher",
        output="screen",
    )

    # Obstacle distance bridge: converts nvblox 2D ESDF slice to
    # MAVLink OBSTACLE_DISTANCE for ArduPilot obstacle avoidance.
    # Skipped when nvblox is disabled — its source topic wouldn't exist.
    obstacle_distance_bridge = ExecuteProcess(
        cmd=[
            "python3",
            "/workspaces/isaac_ros-dev/edge_core/ros/obstacle_distance_bridge.py",
            "--host",
            "172.17.0.1",
            "--port",
            "8000",
            "--rate",
            "5.0",
            "--buffer",
            "2.0",
            "--topic",
            "/nvblox_node/combined_dynamic_map_slice",
        ],
        name="obstacle_distance_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_nvblox")),
    )

    # Nav2 stack
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": nav2_params_file,
            "autostart": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
    )

    nav2_goal_bridge = ExecuteProcess(
        cmd=[
            "python3",
            "/workspaces/isaac_ros-dev/edge_core/ros/nav2_goal_bridge.py",
            "--host",
            "172.17.0.1",
            "--port",
            "8000",
            "--rate",
            "2.0",
        ],
        name="nav2_goal_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
    )

    # Task 1 target localizer: HSV circle detection for capture operations
    target_localizer = ExecuteProcess(
        cmd=[
            "/bin/bash",
            "-c",
            ". /opt/ros/humble/setup.bash && . /workspaces/isaac_ros-dev/install/setup.bash && PYTHONPATH=/workspaces/isaac_ros-dev/edge_core/target_localizer:$PYTHONPATH exec python3 -m target_localizer.target_localizer_node --ros-args -p output_dir:=/workspaces/isaac_ros-dev/data/task1_captures -p team_name:=MAD -r /zed2i/zed_node/rgb/image_rect_color:=/zed/zed_node/rgb/color/rect/image -r /zed2i/zed_node/depth/depth_registered:=/zed/zed_node/depth/depth_registered -r /zed2i/zed_node/rgb/camera_info:=/zed/zed_node/rgb/color/rect/camera_info",
        ],
        name="target_localizer",
        output="screen",
    )

    foxglove_bridge = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "foxglove_bridge",
            "foxglove_bridge",
            "--ros-args",
            "-p",
            "port:=8765",
            "-p",
            "address:=0.0.0.0",
            "-p",
            "send_buffer_limit:=10000000",
        ],
        name="foxglove_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_foxglove")),
    )

    # Static TF alias: ZED URDF uses "zed_left_camera_frame_optical" but
    # the ZED node publishes images with frame_id "zed_left_camera_optical_frame".
    optical_frame_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="optical_frame_alias",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "zed_left_camera_frame_optical",
            "--child-frame-id", "zed_left_camera_optical_frame",
        ],
    )

    # --- SPLIT: ZED and nvblox in separate composable containers ---
    #
    # Previously both lived in one component_container_mt (via zed_example.launch.py).
    # A nvblox SIGABRT (CUDA OOM) would kill the entire container, taking ZED
    # and the video bridge down with it. Now each gets its own process — nvblox
    # can crash and ZED keeps streaming/publishing VIO.

    # Robot state publisher so ZED URDF is available on /robot_description.
    zed_state_publisher = Node(
        package="robot_state_publisher",
        namespace="zed",
        executable="robot_state_publisher",
        name="zed_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command([
                "xacro", " ", xacro_path, " ",
                "camera_name:=zed camera_model:=zed2",
            ])
        }],
        condition=IfCondition(LaunchConfiguration("enable_nvblox")),
    )

    # ZED standalone container — isolated from nvblox CUDA context.
    zed_split_container = ComposableNodeContainer(
        name="zed_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="zed_components",
                namespace="zed",
                name="zed_node",
                plugin="stereolabs::ZedCamera",
                parameters=[
                    # Priority order: later files override earlier.
                    # common_stereo.yaml is patched by startup script at boot.
                    zed_stereo_config,
                    nvblox_zed_common_config,
                    nvblox_zed2_config,
                    {
                        "general.camera_name": "zed",
                        "general.camera_model": "zed2",
                    },
                ],
            )
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_nvblox")),
    )

    # nvblox in its own separate container — subscribes to ZED topics via DDS.
    # If this container crashes, ZED keeps running.
    nvblox_split_container = ComposableNodeContainer(
        name="nvblox_node_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                name="nvblox_node",
                package="nvblox_ros",
                plugin="nvblox::NvbloxNode",
                remappings=[
                    ("camera_0/depth/image", "/zed/zed_node/depth/depth_registered"),
                    ("camera_0/depth/camera_info", "/zed/zed_node/depth/camera_info"),
                    ("camera_0/color/image", "/zed/zed_node/rgb/color/rect/image"),
                    ("camera_0/color/camera_info", "/zed/zed_node/rgb/color/rect/camera_info"),
                    ("pose", "/zed/zed_node/pose"),
                ],
                parameters=[
                    nvblox_base_config,
                    nvblox_zed_config,
                    {"num_cameras": 1, "use_lidar": False},
                ],
            )
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_nvblox")),
    )

    # ZED standalone only (no nvblox) — when enable_nvblox:=false.
    zed_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_only_launch),
        launch_arguments={
            "zed_camera_model": "zed2",
            "run_standalone": "True",
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("enable_nvblox")),
    )

    return LaunchDescription(
        [
            enable_od_arg,
            enable_nav2_arg,
            enable_foxglove_arg,
            enable_nvblox_arg,
            # Split ZED + nvblox (enable_nvblox:=true)
            zed_state_publisher,
            zed_split_container,
            nvblox_split_container,
            # ZED only (enable_nvblox:=false)
            zed_only,
            # Common nodes (always run)
            optical_frame_alias,
            target_localizer,
            servo_tf_publisher,
            obstacle_distance_bridge,
            nav2_launch,
            nav2_goal_bridge,
            foxglove_bridge,
        ]
    )
