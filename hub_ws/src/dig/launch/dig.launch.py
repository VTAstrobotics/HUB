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

    

    spawn_dig_motor = Node(package = "motor_control",
    executable = "kraken_control_node",
    name = "kraken_control_node",
    parameters=[{"motor_name": "dig_motor_teleop"},
                {"can_interface": "can1"},
                {"can_id": 11},
                {"control_topic": "/dig_motor_teleop/control"},
                {"status_topic": "/dig_motor_teleop/status"},
                {"health_topic": "/dig_motor_teleop/health"}],
                #TODO add KG Vals and such
    arguments=["--ros-args",
               "-r",
               "__node:=dig_motor_teleop_controller"]
    )



    # foxglove_studio = Node(
    #     package="foxglove_bridge",
    #     executable="foxglove_bridge",
    #     name="foxglove_bridge"
    # )

    return LaunchDescription([
        spawn_dig_teleop,
        spawn_dig_motor
        # foxglove_studio,
    ])

