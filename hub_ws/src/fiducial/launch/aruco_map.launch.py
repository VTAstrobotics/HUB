# launch/tag_static_transforms.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    pkg_share = get_package_share_directory('fiducial')
    config_file = os.path.join(pkg_share, 'config', 'aruco_map.yaml')

    with open(config_file, 'r') as f:
        data = yaml.safe_load(f)

    parent_frame = 'map' 
    nodes = []

    for tag_id, pose in data['tags'].items():
        x, y, z, roll, pitch, yaw = pose #grab yaml (i don't know how useful this actually is)
        child_frame = f'aruco_tag_{tag_id}'
        nodes.append( #(condisider map to be  middle of arena)
            Node( 
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tag_{tag_id}',
                arguments=[
                    '--x', str(x),
                    '--y', str(y),
                    '--z', str(z),
                    '--roll', str(roll),
                    '--pitch', str(pitch),
                    '--yaw', str(yaw),
                    '--frame-id', parent_frame,
                    '--child-frame-id', child_frame,
                ],
                output='screen',
            )
        )

    return LaunchDescription(nodes)