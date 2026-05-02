import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    spawn_drive = Node(package="drive",            
        executable="drive_node",
        name="drive_node",
        parameters=[{"use_sim_time": False}]
    )
    
    spawn_left_motor = Node(package = "motor_control",
    executable = "sparkmax_control_node",
    name = "sparkmax_control_node",
    parameters=[{"motor_name": "left_motor"},
                {"can_interface": "can1"},
                {"can_id": 11},
                {"control_topic": "/front_left/control"},
                {"status_topic": "/front_left/status"},
                {"health_topic": "/front_left/health"},
                {"kP": 0.00008958}, #Test Value
                {"kI": 0.0},
                {"kD": 0.0001}, 
                {"kF": 0.00008},  
                ],
    arguments=["--ros-args",
               "-r",
               "__node:=left_motor_controller"]
    )

    spawn_right_motor = Node(package = "motor_control",
    executable = "sparkmax_control_node",
    name = "sparkmax_control_node",
    parameters=[{"motor_name": "front_right"},
                {"can_interface": "can1"},
                {"can_id": 10},
                {"control_topic": "/front_right/control"},
                {"status_topic": "/front_right/status"},
                {"health_topic": "/front_right/health"},
                {"kP": 0.00008}, 
                {"kI": 0.0},
                {"kD": 0.0001}, 
                {"kF": 0.00008},  
                ],
    arguments=["--ros-args",
               "-r",
               "__node:=right_motor_controller"]
    )

    return LaunchDescription([
        spawn_drive,
        spawn_left_motor,
        spawn_right_motor
    ])
