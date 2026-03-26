from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_detector')
    camera_info_url = 'package://aruco_detector/config/camera_info.yaml'

    cameras = [
        {'name': 'camera_front', 'device': '/dev/video0'},
        {'name': 'camera_rear',  'device': '/dev/video1'},
        {'name': 'camera_left',  'device': '/dev/video2'},
        {'name': 'camera_right', 'device': '/dev/video3'},
    ]

    nodes = []

    for cam in cameras: #spin up n camera nodes with n aruco listeners
        cam_name = cam['name']
        video_device = cam['device']

        nodes.append(
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name=f'{cam_name}_usb_cam',
                namespace=cam_name,
                output='screen',
                parameters=[{ #half of this is not needed
                    'video_device': video_device,
                    'framerate': 30.0,
                    'io_method': 'mmap',
                    'frame_id': f'{cam_name}_frame',
                    'pixel_format': 'yuyv',
                    'av_device_format': 'YUV422P',
                    'image_width': 640,
                    'image_height': 480,
                    'camera_name': cam_name,
                    'camera_info_url': camera_info_url,
                    'brightness': -1,
                    'contrast': -1,
                    'saturation': -1,
                    'sharpness': -1,
                    'gain': -1,
                    'auto_white_balance': True,
                    'white_balance': 4000,
                    'autoexposure': True,
                    'exposure': 100,
                    'autofocus': False,
                    'focus': -1,
                }]
            )
        )

        nodes.append(
            Node(
                package='aruco_detector',
                executable='aruco_detector',
                name=f'{cam_name}_aruco_detector',
                output='screen',
                parameters=[{
                    'camera_name': cam_name
                }]
            )
        )

    return LaunchDescription(nodes)