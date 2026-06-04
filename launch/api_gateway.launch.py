"""Bring up the JSON bridge.

Typical launch order:

    1) ros2 launch openarm_bimanual_moveit_config demo.launch.py
    2) ros2 launch openarm_perception perception.launch.py     # optional
    3) ros2 launch openarm_skills skills.launch.py
    4) ros2 launch openarm_api    api_gateway.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("openarm_api"), "config", "api_config.yaml"
    )
    enable_http = LaunchConfiguration("enable_http")
    api_log_args = [
        "--ros-args",
        "--log-level",
        PythonExpression([
            "'openarm_api:=debug' if '",
            LaunchConfiguration("openarm_debug"),
            "' == 'true' else 'openarm_api:=info'",
        ]),
        "--log-level", "rcl:=warn",
        "--log-level", "rmw_fastrtps_cpp:=warn",
        "--log-level", "rclcpp:=info",
        "--log-level", "openarm_api_io:=warn",
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_http", default_value="false",
            description="Also launch FastAPI/WS gateway on http_port"),
        DeclareLaunchArgument(
            "openarm_debug", default_value="false",
            description="Set openarm_api logger to debug ([DBG] traces only)"),
        Node(
            package="openarm_api",
            executable="json_bridge_node",
            name="openarm_api",
            output="screen",
            parameters=[cfg, {"enable_http": enable_http}],
            arguments=api_log_args,
        ),
    ])
