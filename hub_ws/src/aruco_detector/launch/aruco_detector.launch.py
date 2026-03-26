from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_detector')
    rviz_config_file = os.path.join(pkg_share, 'config', 'display.rviz')
    usb_cam_config_file = os.path.join(pkg_share, 'config', 'usb_cam_params.yaml')

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[usb_cam_config_file],
            output='screen'
        ),
        Node(
            package='aruco_detector',
            executable='aruco_detector',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])