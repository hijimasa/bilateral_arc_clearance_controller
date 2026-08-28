"""Launch the BAC evaluation filter with configurable topic remappings."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("bilateral_arc_clearance_controller"),
        "config",
        "bac_filter.yaml",
    )

    arguments = [
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("scan", default_value="/scan"),
        DeclareLaunchArgument("odom", default_value="/odom"),
        DeclareLaunchArgument("cmd_vel_in", default_value="/nav_cmd_vel"),
        DeclareLaunchArgument("cmd_vel_out", default_value="/cmd_vel"),
    ]
    node = Node(
        package="bilateral_arc_clearance_controller",
        executable="bac_filter_node",
        name="bac_filter",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[
            ("scan", LaunchConfiguration("scan")),
            ("odom", LaunchConfiguration("odom")),
            ("cmd_vel_in", LaunchConfiguration("cmd_vel_in")),
            ("cmd_vel_out", LaunchConfiguration("cmd_vel_out")),
        ],
    )
    return LaunchDescription(arguments + [node])
