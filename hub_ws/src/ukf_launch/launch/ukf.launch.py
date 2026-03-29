from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share_dir = get_package_share_directory("ukf_launch")
    ukf_config_file = os.path.join(package_share_dir, "config", "ukf_config.yaml")
    aruco_ukf_config_file = os.path.join(
        package_share_dir, "config", "ukf_config_aruco.yaml"
    )
    map_ukf_config_file = os.path.join(
        package_share_dir, "config", "ukf_config_map.yaml"
    )

    ukf_node = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf_node",
        output="screen",
        parameters=[{"use_sim_time": False}, ukf_config_file],
    )

    aruco_ukf_node = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf_node_aruco",
        output="screen",
        parameters=[{"use_sim_time": False}, aruco_ukf_config_file],
        remappings=[("odometry/filtered", "odometry/filtered_aruco")],
    )

    aruco_ukf_node = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf_node_map",
        output="screen",
        parameters=[{"use_sim_time": False}, map_ukf_config_file],
        remappings=[("odometry/filtered", "odometry/filtered_map")],
    )

    return LaunchDescription([ukf_node, aruco_ukf_node, aruco_ukf_node])
