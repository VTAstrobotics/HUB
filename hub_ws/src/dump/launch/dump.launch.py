import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    spawn_dump_auto = Node(package="dump",            
        executable="dump_auto_action_server",
        name="dump_auto_action_server",
        parameters=[]
    )

    spawn_dump_door_motor = Node(package = "motor_control",
    executable = "kraken_control_node",
    name = "kraken_control_node",
    parameters=[{"motor_name": "dump_door"},
                {"can_interface": "can1"},
                {"can_id": 11},
                {"control_topic": "/dump_door/control"},
                {"status_topic": "/dump_door/status"},
                {"health_topic": "/dump_door/health"}],
    arguments=["--ros-args",
               "-r",
               "__node:=dump_door_controller"]
    )

    spawn_dump_linear_actuator = Node(package = "motor_control",
    executable = "sparkmax_control_node",
    name = "sparkmax_control_node",
    parameters=[{"motor_name": "dump_linear_actuator"},
                {"can_interface": "can1"},
                {"can_id": 12}, 
                {"control_topic": "/dump_linear_actuator/control"},
                {"status_topic": "/dump_linear_actuator/status"},
                {"health_topic": "/dump_linear_actuator/health"}],
    arguments=["--ros-args",
               "-r",
               "__node:=dump_linear_actuator_controller"]
    )


    foxglove_studio = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge"
    )

    return LaunchDescription([
        spawn_dump_auto,
        spawn_dump_door_motor,
        spawn_dump_linear_actuator,
        foxglove_studio,
    ])

