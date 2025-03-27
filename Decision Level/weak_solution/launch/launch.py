from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    weak_solution_node = Node(
        package="weak_solution",
        name="weak_solution_node",
        executable="weak_solution_node",
        namespace="",
        output="screen",
    )

    return LaunchDescription([weak_solution_node])
