#!/usr/bin/env python3
"""
NOMAD Task 2 Launch File - Direct ROS2 to MAVLink Architecture

This launch file starts the complete navigation stack:
1. ZED Camera (VIO, depth, object detection)
2. VIO Bridge (ZED odom -> MAVROS vision_pose)  
3. MAVROS (MAVLink to/from ArduPilot)
4. Nav2 Stack (path planning, obstacle avoidance)

CRITICAL: Nav2 /cmd_vel is remapped to /mavros/setpoint_velocity/cmd_vel_unstamped
so velocity commands go directly to ArduPilot without middleware.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParametersFromFile
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Paths
    workspace = '/workspaces/isaac_ros-dev'
    nav2_params_file = os.path.join(workspace, 'nav2_mavros_params.yaml')
    
    # ZED camera launch (assumes zed_wrapper is installed)
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'serial_number': '0',
            'publish_urdf': 'true',
            'publish_tf': 'true',
            'publish_map_tf': 'false',
            'base_frame': 'base_link',
            'cam_pos_x': '0.0',
            'cam_pos_y': '0.0', 
            'cam_pos_z': '0.1',
            'cam_roll': '0.0',
            'cam_pitch': '0.0',
            'cam_yaw': '0.0',
        }.items()
    )
    
    # VIO Bridge - converts ZED odom to MAVROS vision_pose
    vio_bridge_node = Node(
        package='nomad_ros',  # Or use 'python3' executable if not in package
        executable='vio_bridge.py',
        name='vio_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    
    # For running vio_bridge as standalone script (if not packaged):
    vio_bridge_standalone = Node(
        name='vio_bridge',
        executable='python3',
        arguments=[os.path.join(workspace, 'vio_bridge.py')],
        output='screen',
    )
    
    # MAVROS - connects to ArduPilot on Cube Orange
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': '/dev/ttyACM0:921600',
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
        }],
    )
    
    # Nav2 Controller Server with velocity remapping to MAVROS
    # CRITICAL: This remaps /cmd_vel -> /mavros/setpoint_velocity/cmd_vel_unstamped
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/mavros/setpoint_velocity/cmd_vel_unstamped'),
            ('/odom', '/zed/zed_node/odom'),
        ],
    )
    
    # Nav2 Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )
    
    # Nav2 Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/mavros/setpoint_velocity/cmd_vel_unstamped'),
        ],
    )
    
    # Nav2 BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )
    
    # Nav2 Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ]
        }],
    )
    
    # Velocity Smoother with MAVROS remapping
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel_smoothed', '/mavros/setpoint_velocity/cmd_vel_unstamped'),
            ('/odom', '/zed/zed_node/odom'),
        ],
    )
    
    # Local Costmap (uses ZED point cloud)
    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        namespace='local_costmap',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )
    
    # Global Costmap
    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap', 
        namespace='global_costmap',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # Core perception
        # zed_launch,  # Uncomment if not already running
        vio_bridge_standalone,
        mavros_node,
        
        # Nav2 stack
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        local_costmap,
        global_costmap,
        lifecycle_manager,
    ])
