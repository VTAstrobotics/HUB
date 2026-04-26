import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    spawn_dig_teleop = Node(package="dig",            
        executable="dig_teleop_node",
        name="dig_teleop_node",
        parameters=[]
    )

    spawn_dig_auto_node = Node(package="dig",            
        executable="dig_action_server",
        name="dig_action_server",
        parameters=[]
    )

    

    spawn_dig_motor = Node(package = "motor_control",
    executable = "kraken_control_node",
    name = "kraken_control_node",
    parameters=[{"motor_name": "dig_motor"},
                {"can_interface": "can1"},
                {"can_id": 23},
                {"control_topic": "/dig_motor/control"},
                {"status_topic": "/dig_motor/status"},
                {"health_topic": "/dig_motor/health"},
                {"encoder_canID": 24}, 
                {"kG": 0.08}, 
                {"kP": 18.0}, 
                {"arm_cosine": True},
                # {"closed_loop_ramp_rate": 0.8},
                {"brake": True}, 
                ],
                #TODO add KG Vals and such
    arguments=["--ros-args",
               "-r",
               "__node:=dig_motor_teleop_controller"]
    )

    spawn_dig_motor = Node(package = "motor_control",
    executable = "kraken_control_node",
    name = "kraken_control_node",
    parameters=[{"motor_name": "dig_motor"},
                {"can_interface": "can1"},
                {"can_id": 25},
                {"control_topic": "/dig_motor/control"},
                {"status_topic": "/dig_motor/status"},
                {"health_topic": "/dig_motor/health"},
                {"leader_id": 23},
                {"brake": True}, 
                ],
                #TODO add KG Vals and such
    arguments=["--ros-args",
               "-r",
               "__node:=dig_motor_teleop_controller"]
    )

    return LaunchDescription([
        spawn_dig_teleop,
        spawn_dig_motor,
        spawn_dig_auto_node
        # foxglove_studio,
    ])

