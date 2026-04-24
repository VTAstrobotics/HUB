from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch.conditions import IfCondition, UnlessCondition
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Locate the other packages' share directories
    distributor_share = get_package_share_directory("distributor")
    dig_share = get_package_share_directory("dig")
    drive_share = get_package_share_directory("drive")
    dump_share = get_package_share_directory("dump")

    dig_launch = os.path.join(dig_share, "launch", "dig_teleop.launch.py")
    drive_launch = os.path.join(drive_share, "launch", "old_drive.launch.py")
    dump_launch = os.path.join(dump_share, "launch", "dump.launch.py")

    spawn_distributor_node = Node(
        package="distributor",
        executable="distributor_node",
        name="distributor_node",
    )

    return LaunchDescription(
        [
            ExecuteProcess(
            cmd=['bash', '../../../launch_scripts/can_startup.sh'],
            # prefix=['sudo'],
            output='screen'
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(dig_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(drive_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(dump_launch)),
            spawn_distributor_node,
        ]
    )
