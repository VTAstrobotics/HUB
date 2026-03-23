from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("motor_control")
    config_file = os.path.join(pkg_share, "config", "kraken_params.yaml")

    # List each node name defined in the YAML
    motor_nodes = ["kraken_front_left", "kraken_front_right"]

    nodes = [
        Node(
            package="motor_control",
            executable="kraken_control_node",
            name=node_name,
            output="screen",
            parameters=[config_file],
        )
        for node_name in motor_nodes
    ]

    return LaunchDescription(nodes)
