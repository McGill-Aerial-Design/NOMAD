# NOMAD ZED + nvblox + Nav2 + Custom Object Detection Launch File
#
# Wraps the standard zed_example.launch.py and adds:
# - camera=zed2 for the ZED 2i (zed_example launch expects zed2 token)
# - Custom YOLO26 object detection via ZED SDK custom OD pipeline
# - Servo TF publisher (servo_mount -> camera_link dynamic transform)
# - Static TF: base_link -> servo_mount (mounting offset)
# - Obstacle distance bridge (nvblox ESDF -> MAVLink OBSTACLE_DISTANCE)
# - Nav2 stack with nvblox costmap for Jetson-side obstacle avoidance (optional)
# - Nav2 goal bridge (Edge Core API -> Nav2 actions) (optional)
#
# The ZED SDK loads the ONNX model, auto-converts to TensorRT on first run,
# and publishes 3D detections to /zed/zed_node/obj_det/objects.
#
# TF Tree (complete chain):
#   map -> odom -> zed_camera_link -> zed_camera_center -> zed_left_camera_frame
#                                                          -> zed_left_camera_frame_optical
#                                                             -> zed_left_camera_optical_frame (alias)
#
# NOTE: ZED SDK object detection params (od_enabled, custom_onnx_file, model, etc.)
# are read at initialization time and are NOT dynamically reconfigurable.
# The custom YOLO26 config is merged into zed_common.yaml BEFORE launch
# by the api.py launch script, so ros2 param load is not needed here.
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
#   ros2 launch /workspaces/isaac_ros-dev/config/launch/nomad_zed_nvblox.launch.py enable_nav2:=true

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nvblox_bringup_dir = get_package_share_directory("nvblox_examples_bringup")
    zed_example_launch = os.path.join(
        nvblox_bringup_dir, "launch", "zed_example.launch.py"
    )
    # ZED-only launch (loads only the ZED composable node, no nvblox).
    # Used when enable_nvblox:=false so we keep video, ros_http_bridge,
    # target_localizer, etc., without paying for nvblox VRAM/CUDA pressure.
    zed_only_launch = os.path.join(
        nvblox_bringup_dir, "launch", "sensors", "zed.launch.py"
    )

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
        description="Load the nvblox composable node alongside ZED (true) or run ZED only with the rest of the bringup intact (false). Set false to keep video / ros_http_bridge / target_localizer running without nvblox VRAM and CUDA pressure.",
    )

    # NOTE: Do NOT patch pub_downscale_factor to 1.0 (720p).
    # ZED 360p (default downscale 2.0) uses ~75% less GPU memory than 720p.
    # On 8GB Jetson Orin Nano, 720p depth causes cudaErrorIllegalAddress
    # when nvblox allocates GPU memory for depth integration.

    # Servo TF publisher: publishes servo_mount -> camera_link at 50 Hz
    # reflecting the current servo pitch angle (TF-001), and also
    # publishes odom -> base_link by inverting the camera odom pose
    # so the full TF chain is connected: odom -> base_link -> servo_mount -> camera_link
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
    # MAVLink OBSTACLE_DISTANCE for ArduPilot obstacle avoidance (NV-008).
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

    # Nav2 stack: full navigation with nvblox costmap for obstacle avoidance
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

    # Nav2 goal bridge: polls Edge Core for navigation goals, sends to Nav2
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
    # Runs independently of ZED OD mode since we disabled ZED OD to prevent crashes
    target_localizer = ExecuteProcess(
        cmd=[
            "/bin/bash",
            "-c",
            ". /opt/ros/humble/setup.bash && . /workspaces/isaac_ros-dev/install/setup.bash && PYTHONPATH=/workspaces/isaac_ros-dev/edge_core/target_localizer:$PYTHONPATH exec python3 -m target_localizer.target_localizer_node --ros-args -p output_dir:=/workspaces/isaac_ros-dev/data/task1_captures -p team_name:=MAD -r /zed2i/zed_node/rgb/image_rect_color:=/zed/zed_node/rgb/color/rect/image -r /zed2i/zed_node/depth/depth_registered:=/zed/zed_node/depth/depth_registered -r /zed2i/zed_node/rgb/camera_info:=/zed/zed_node/rgb/color/rect/camera_info",
        ],
        name="target_localizer",
        output="screen",
    )

    # Foxglove bridge: exposes all ROS2 topics via WebSocket for Foxglove Studio
    # Connect Foxglove Studio to ws://<jetson-ip>:8765
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
    # nvblox needs to look up the image frame in TF, so we bridge the gap
    # with an identity transform.
    optical_frame_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="optical_frame_alias",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "zed_left_camera_frame_optical",
            "--child-frame-id",
            "zed_left_camera_optical_frame",
        ],
    )

    # ZED + nvblox in one composable container (default) ...
    zed_with_nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_example_launch),
        launch_arguments={
            "camera": "zed2",
            "enable_od": LaunchConfiguration("enable_od"),
            "run_rviz": "false",  # Headless Jetson — no display for rviz
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_nvblox")),
    )
    # ... or just ZED, in its own standalone container, when enable_nvblox:=false.
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
            zed_with_nvblox,
            zed_only,
            optical_frame_alias,
            target_localizer,
            servo_tf_publisher,
            obstacle_distance_bridge,
            nav2_launch,
            nav2_goal_bridge,
            foxglove_bridge,
        ]
    )
