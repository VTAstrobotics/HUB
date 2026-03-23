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
        parameters=[{"use_sim_time": False},
                    ]
    )
    
    spawn_left_motor = Node(package = "motor_control",
    executable = "kraken_control_node",
    name = "kraken_control_node",
    parameters=[{"motor_name": "left_front"},
                {"can_interface": "can1"},
                {"can_id": 48},
                {"control_topic": "/left_front/control"},
                {"status_topic": "/left_front/status"},
                {"health_topic": "/left_front/health"}],
    arguments=["--ros-args",
               "-r",
               "__node:=left_motor_controller"]
    )
    spawn_left_back__motor = Node(package = "motor_control",
    executable = "kraken_control_node",
    name = "kraken_control_node",
    parameters=[{"motor_name": "left_back"},
                {"can_interface": "can1"},
                {"can_id": 44},
                {"control_topic": "/left_back/control"},
                {"status_topic": "/left_back/status"},
                {"health_topic": "/left_back/health"}],
    arguments=["--ros-args",
            "-r",
            "__node:=left_motor_controller"]
)


    spawn_right_motor = Node(package = "motor_control",
    executable = "kraken_control_node",
    name = "kraken_control_node",
    parameters=[{"motor_name": "right_front"},
                {"can_interface": "can1"},
                {"can_id": 46},
                {"control_topic": "/right_front/control"},
                {"status_topic": "/right_front/status"},
                {"health_topic": "/right_front/health"}],
    arguments=["--ros-args",
               "-r",
               "__node:=right_motor_controller"]
    )
    
    spawn_right_back_motor = Node(package = "motor_control",
    executable = "kraken_control_node",
    name = "kraken_control_node",
    parameters=[{"motor_name": "right_back"},
                {"can_interface": "can1"},
                {"can_id": 45},
                {"control_topic": "/right_back/control"},
                {"status_topic": "/right_back/status"},
                {"health_topic": "/right_back/health"}],
    arguments=["--ros-args",
               "-r",
               "__node:=right_motor_controller"]
    )

     # calibration command

    # each camera will require these 2 nodes to be started for it to work properly.
    webcam_v4l2 = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="Front_Camera_Driver",
        output="screen",
        namespace="front_camera",
        parameters=[{
            "video_device": "/dev/v4l/by-id/usb-046d_Brio_101_2450APR8ZF68-video-index0",
            "image_size": [800, 600],
    #    {
            "image_raw.ffmpeg.qmax": 1,  # high quality
            "image_raw.ffmpeg.bit_rate": 1000000,  # required!
            "image_raw.ffmpeg.gop_size": 1,
            "image_raw.ffmpeg.encoder": "libx264",
        # },
        }]
        # using the video device from v4l means we access the same camera each time and the driver exposes what camera is active on a node via the 
        # video device parameter via ros2. This lets us actually access the camera easily compared to /dev/video* ids.
    )
    aruco_webcam = Node(
        package="pose_estimation",
        executable="v4l2_aruco_node",
        name="webcam_camera_aruco",
        output="screen",
        namespace="front_camera",
        parameters=[{
            "camera_name": "usb-046d_Brio_101_2450APR8ZF68-video-index0",
            "aruco_topic": "/front_camera/image_raw"
        }]
        # using the video device from v4l means we access the same camera each time and the driver exposes what camera is active on a node via the 
        # video device parameter via ros2. This lets us actually access the camera easily compared to /dev/video* ids.
    )

    foxglove_studio = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge"
    )

    return LaunchDescription([
        # spawn_drive,
        # spawn_left_motor,
        # spawn_right_motor,
        # spawn_left_back__motor,
        # spawn_right_back_motor,
        webcam_v4l2,
        # foxglove_studio,
        aruco_webcam
    ])

