# NOMAD ZED + nvblox + Custom Object Detection Launch File
#
# Wraps the standard zed_example.launch.py and adds:
# - camera=zed2 for the ZED 2i
# - Custom YOLO26 object detection via ZED SDK custom OD pipeline
# - Servo TF publisher (servo_mount -> camera_link dynamic transform)
# - Static TF: base_link -> servo_mount (mounting offset)
#
# The ZED SDK loads the ONNX model, auto-converts to TensorRT on first run,
# and publishes 3D detections to /zed/zed_node/obj_det/objects.
#
# TF Tree (complete chain):
#   map -> odom -> base_link -> servo_mount -> camera_link
#                                              -> zed2i_left_camera_optical_frame
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

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nvblox_bringup_dir = get_package_share_directory('nvblox_examples_bringup')
    zed_example_launch = os.path.join(
        nvblox_bringup_dir, 'launch', 'zed_example.launch.py'
    )

    # Custom OD config path (mounted inside the container)
    custom_od_config = '/workspaces/isaac_ros-dev/config/custom_circle_detection.yaml'

    enable_od_arg = DeclareLaunchArgument(
        'enable_od',
        default_value='true',
        description='Enable ZED custom object detection (YOLO26 circle detection)',
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

    return LaunchDescription([
        enable_od_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_example_launch),
            launch_arguments={
                'camera': 'zed2',
                'od_enabled': LaunchConfiguration('enable_od'),
                'custom_object_detection_config_path': custom_od_config,
            }.items(),
        ),
        servo_tf_publisher,
    ])
