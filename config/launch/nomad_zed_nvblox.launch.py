# NOMAD nvblox-only launch file.
#
# The ZED wrapper, Task 1 target_localizer, optical-frame alias, and servo TF
# publisher are owned by scripts/services/zed_wrapper.sh. This launch file is
# intentionally limited to optional nvblox mapping and its obstacle-distance
# bridge so starting/stopping nvblox cannot cycle the camera or duplicate helper
# nodes. Competition deployments can leave nomad-nvblox.service disabled.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nvblox_bringup_dir = get_package_share_directory("nvblox_examples_bringup")

    nvblox_base_config = (
        f"{nvblox_bringup_dir}/config/nvblox/nvblox_base.yaml"
    )
    nvblox_zed_config = (
        f"{nvblox_bringup_dir}/config/nvblox/specializations/nvblox_zed.yaml"
    )

    enable_nvblox_arg = DeclareLaunchArgument(
        "enable_nvblox",
        default_value="true",
        description="Start the single optional nvblox mapper.",
    )

    nvblox_container = ComposableNodeContainer(
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

    return LaunchDescription(
        [
            enable_nvblox_arg,
            nvblox_container,
            obstacle_distance_bridge,
        ]
    )
