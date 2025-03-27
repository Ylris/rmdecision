from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    node_params = os.path.join(
        get_package_share_directory("rm_decision"), "config", "node_params.yaml"
    )

    rm_decision_node = Node(
        package="rm_decision",
        name="rm_decision_node",
        executable="rm_decision_node",
        namespace="",
        output="screen",
        parameters=[node_params],
    )

    return LaunchDescription([rm_decision_node])
