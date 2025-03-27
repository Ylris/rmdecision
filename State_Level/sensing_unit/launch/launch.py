from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    node_params = os.path.join(
        get_package_share_directory("sensing_unit"), "config", "node_params.yaml"
    )
    
    sensing_unit_node = Node(
        package="sensing_unit",
        name="sensing_unit_node",
        executable="sensing_unit_node",
        namespace="",
        output="screen",
        parameters=[node_params],
    )

    return LaunchDescription([sensing_unit_node])
