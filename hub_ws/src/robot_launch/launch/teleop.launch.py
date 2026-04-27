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

    foxglove_studio = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge"
    )

    try: 
        zed_wrapper_dir = get_package_share_directory("zed_wrapper")
        zed_launch =  os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
        found_zed = True
    except:
        found_zed = False


    nodes = [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(dig_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(drive_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(dump_launch)),
            spawn_distributor_node,
            foxglove_studio,
        ]

    if found_zed:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(zed_launch),
                launch_arguments={
                    'camera_model': 'zedm'
                }.items()
            )
    )

    return LaunchDescription(
            nodes
    )
