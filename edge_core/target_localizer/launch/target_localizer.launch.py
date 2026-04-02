"""
Launch file for the target localizer node.

Usage:
  ros2 launch target_localizer target_localizer.launch.py

  # Override building params at launch:
  ros2 launch target_localizer target_localizer.launch.py \
      building_lat:=45.3220 building_lon:=-75.7600 \
      building_length:=12.0 building_width:=8.0 \
      building_height:=5.0 building_orientation:=45.0

  # With YOLO landmark model:
  ros2 launch target_localizer target_localizer.launch.py \
      yolo_model:=/home/mad/models/landmarks.onnx
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare launch arguments with defaults
    args = [
        DeclareLaunchArgument('building_lat', default_value='0.0',
                              description='Building center latitude'),
        DeclareLaunchArgument('building_lon', default_value='0.0',
                              description='Building center longitude'),
        DeclareLaunchArgument('building_length', default_value='10.0',
                              description='Building length along orientation axis (m)'),
        DeclareLaunchArgument('building_width', default_value='6.0',
                              description='Building width perpendicular to orientation (m)'),
        DeclareLaunchArgument('building_height', default_value='5.0',
                              description='Building height (m)'),
        DeclareLaunchArgument('building_orientation', default_value='0.0',
                              description='Building long-axis heading (deg CW from north)'),
        DeclareLaunchArgument('team_name', default_value='MAD',
                              description='Team name for output file naming'),
        DeclareLaunchArgument('output_dir', default_value='/home/mad/targets',
                              description='Directory for output .txt and debug files'),
        DeclareLaunchArgument('yolo_model', default_value='',
                              description='Path to YOLO ONNX model for landmark detection'),
        DeclareLaunchArgument('dedup_radius', default_value='0.5',
                              description='Deduplication radius in meters'),
    ]

    target_localizer_node = Node(
        package='target_localizer',
        executable='target_localizer_node',
        name='target_localizer',
        output='screen',
        parameters=[{
            'building.center_lat': LaunchConfiguration('building_lat'),
            'building.center_lon': LaunchConfiguration('building_lon'),
            'building.length': LaunchConfiguration('building_length'),
            'building.width': LaunchConfiguration('building_width'),
            'building.height': LaunchConfiguration('building_height'),
            'building.orientation_deg': LaunchConfiguration('building_orientation'),
            'team_name': LaunchConfiguration('team_name'),
            'output_dir': LaunchConfiguration('output_dir'),
            'yolo_model_path': LaunchConfiguration('yolo_model'),
            'dedup_radius_m': LaunchConfiguration('dedup_radius'),
            'landmark_detect_rate_hz': 2.0,
            'auto_landmark_detection': True,
        }],
        remappings=[
            # NOMAD launches ZED with camera:=zed2, so topics are /zed/zed_node/...
            ('/zed2i/zed_node/rgb/image_rect_color', '/zed/zed_node/rgb/image_rect_color'),
            ('/zed2i/zed_node/depth/depth_registered', '/zed/zed_node/depth/depth_registered'),
            ('/zed2i/zed_node/rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
        ],
    )

    return LaunchDescription(args + [target_localizer_node])
