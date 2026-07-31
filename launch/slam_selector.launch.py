from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("rover_slam_selector"),
        "config",
        "slam_selector.yaml",
    )

    return LaunchDescription([
        Node(
            package="rover_slam_selector",
            executable="slam_selector",
            name="slam_selector",
            output="screen",
            parameters=[config_file],
        )
    ])
