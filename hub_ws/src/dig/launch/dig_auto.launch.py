import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    spawn_dig_auto = Node(package="dig",            
        executable="dig_action_server",
        name="dig_action_server",
        parameters=[]
    )

    spawn_dig_motor = Node(package = "motor_control",
    executable = "kraken_control_node",
    name = "kraken_control_node",
    parameters=[{"motor_name": "dig_motor"}, #TODO add PID and external encoder parameters
                {"can_interface": "can1"},
                {"can_id": 23},
                {"control_topic": "/dig_motor/control"},
                {"status_topic": "/dig_motor/status"},
                {"health_topic": "/dig_motor/health"},
                {"encoder_canID": 24},
                {"kG": 0.3496}, 
                {"kP": 25.0}, 
                {"arm_cosine": True},
                {"closed_loop_ramp_rate": 0.8},
                {"brake": True},],
    arguments=["--ros-args",
               "-r",
               "__node:=dig_motor_controller"]
    )


    return LaunchDescription([
        spawn_dig_auto,
        spawn_dig_motor
    ])