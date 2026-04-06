from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share_dir = get_package_share_directory("ukf_launch")

    dig_mux_node = Node(
        package="mux", executable="dig_mux_node", name="dig_mux_node", output="screen"
    )

    dump_mux_node = Node(
        package="mux", executable="dump_mux_node", name="dump_mux_node", output="screen"
    )

    return LaunchDescription([dig_mux_node, dump_mux_node])
