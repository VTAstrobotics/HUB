# ROS2 ArUco Detector

This ROS2 package detects an ArUco marker using a USB camera, estimates its pose, and publishes the pose and TF transform for real-time visualization in RViz2.

## Features

- Subscribes to /image_raw and /camera_info for camera input.

- Detects an ArUco marker (ID 0, DICT_6X6_250) using OpenCV 4.5.4.

- Publishes the marker's pose to /aruco_pose and a visualization marker to /aruco_marker.

- Broadcasts the marker’s transform (camera_frame → aruco_frame) to /tf.

- Includes a launch file to start the USB camera, ArUco detector, and RViz2 with a custom configuration.

## Prerequisites

- ROS2 (tested on Humble)

- Ubuntu (tested on 22.04)

- OpenCV 4.5.4 (some API calls have been changed for later version)

- USB camera compatible with the `usb_cam` package

## Installation

1. Clone the repository into your ROS2 workspace:

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/your-username/ros2_aruco_detector.git
    ```

1. Build the workspace:

    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```

1. Ensure the USB camera is configured with a calibration file

## Usage

1. Run the launch file to start the camera, detector, and RViz2:
   
    ```bash
    ros2 launch aruco_detector aruco_detector.launch.py
    ```

1. Display an ArUco marker (ID 64, DICT_6X6_250, 72mm size) on a phone or printed sheet. You can update those ArUco parameters according to your generated ArUco.

1. In RViz2, observe the `aruco_frame` axes and red cube moving relative to the fixed `camera_frame`.

## Notes

1. Ensure the USB camera is calibrated (see ROS2 Camera Calibration Tutorial).

1. For different ArUco markers, update the dictionary or ID in aruco_detector.py.
