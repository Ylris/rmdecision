from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    navigator_node = Node(
        package="navigator",
        name="navigator_node",
        executable="navigator_node",
        namespace="",
        output="screen",
    )

    return LaunchDescription([navigator_node])
