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

    reaper_description_share = get_package_share_directory("reaper_description")

    ukf_launch_share = get_package_share_directory("ukf_launch")

    dig_launch = os.path.join(dig_share, "launch", "dig_teleop.launch.py")
    drive_launch = os.path.join(drive_share, "launch", "old_drive.launch.py")
    dump_launch = os.path.join(dump_share, "launch", "dump.launch.py")
    reaper_description_launch = os.path.join(reaper_description_share, "launch", "launch.py")


    ukf_launch_launch = reaper_description_launch = os.path.join(ukf_launch_share, "launch", "ukf.launch.py")


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


    zed_h264_republisher = Node(
    package='image_transport',
    executable='republish',
    name='zed_h264_republisher',
    arguments=['raw', 'h264'],  # input_transport, output_transport
    remappings=[
        ('in',      '/zed/zed_node/left/image_rect_color'),
        ('out/h264','/zed/zed_node/left/image_rect_color/h264'),
    ],
    parameters=[{
        'h264.bit_rate': 2_000_000,    # 2 Mbps — tune to your bandwidth
        'h264.preset':  'ultrafast',   # minimize encoding latency
        'h264.tune':    'zerolatency',
    }]
)        
    

    base_link_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link_publisher',
        arguments=[
            '--x', '0.2', 
            '--y', '0.0', 
            '--z', '0.3', 
            '--roll', '0.0', 
            '--pitch', '0.0', 
            '--yaw', '0.0', 
            '--frame-id', 'base_link', 
            '--child-frame-id', 'zed_camera_link'
        ]
    )

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    our_nav_share = get_package_share_directory('navigation')
    nav2_params = os.path.join(our_nav_share, 'config', 'nav2_params.yaml')
    nav2_launch = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')


    nodes = [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(dig_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(drive_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(dump_launch)),
            # IncludeLaunchDescription(PythonLaunchDescriptionSource(reaper_description_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(ukf_launch_launch)),
            spawn_distributor_node,
            foxglove_studio,
            zed_h264_republisher,
            base_link_to_camera_link,
            IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_launch), 
                                 launch_arguments={
                                    'use_sim_time': 'false',
                                    'params_file': nav2_params,
                                }.items()
                            ),
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
