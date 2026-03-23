import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory('reaper_description')

    use_rviz = LaunchConfiguration('use_rviz')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    urdf_file = LaunchConfiguration('urdf_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    base_link_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link_publisher',
        arguments=[
            '0.2', '0', '0.3', '0', '0', '0', 'base_link', 'zed_camera_link'
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_robot_state_pub',
            default_value='True',
            description='Start robot_state_publisher'
        ),


        DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(pkg_share, 'urdf', 'URDFassy.urdf'),
            description='Full path to URDF or XACRO'
        ),

        # -------------------------
        # Robot State Publisher
        # -------------------------
        Node(
            condition=IfCondition(use_robot_state_pub),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command([
                        FindExecutable(name='xacro'), ' ',
                        urdf_file
                    ]),
                    value_type=str
                )
            }]
        ),
        base_link_to_camera_link
    ])
