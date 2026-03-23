import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
import cv2
import os
import numpy as np
import math
from packaging import version
from pose_estimation import calibration_manager

class Quaternion:
    w: float
    x: float
    y: float
    z: float

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q 
class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        
        print(cv2.__version__)
        self.declare_parameter("camera_name", "usb-Framework_Laptop_Webcam_Module__2nd_Gen__FRANJBCHA1537100EB-video-index0")
        self.get_logger().info(f"Aruco Tag reader started for camera {self.get_parameter('camera_name')}")
        self.declare_parameter("aruco_topic", "/front_camera/image_raw")

        self.pose_publisher = self.create_publisher(Pose, 'pose', 10)
        self.stop_subscription = self.create_subscription(
            Bool,
            'stop_aruco',
            self.stop_callback,
            10)
        
        self.image_subscription = self.create_subscription(
            Image,
            self.get_parameter("aruco_topic").value,
            self.image_callback,
            10)

        self.detected_publisher = self.create_publisher(Image, f"{self.get_name()}_detected_tags", 2)
            
        self.bridge = CvBridge()
        self.stop = False

    
        if version.parse(cv2.__version__) >= version.parse("4.7.0"):
            self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            self.arucoParams = cv2.aruco.DetectorParameters()
            self.arucoDetector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        else:
            self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            self.arucoParams = cv2.aruco.DetectorParameters_create()




        # Load camera calibration data

        self.cameraMatrix, self.distCoeffs = calibration_manager.load_matrix_distortion(self.get_parameter("camera_name").value)
        self.image_width, self.image_height = calibration_manager.load_width_height(self.get_parameter("camera_name").value)
        
        self.markerLength = 0.06985  # 2 3/4 in tag size
        self.objectPoints = np.array([
            [-self.markerLength / 2, self.markerLength / 2, 0],
            [self.markerLength / 2, self.markerLength / 2, 0],
            [self.markerLength / 2, -self.markerLength / 2, 0],
            [-self.markerLength / 2, -self.markerLength / 2, 0]
        ], dtype=np.float32)
    def stop_callback(self, msg):
        self.stop = msg.data
        if(self.stop):
            self.get_logger().info("Stopped")
    
        else:
            self.get_logger().info("Resumed")

    def image_callback(self, msg):
            try:
                # 1. Convert to BGR (Standard for OpenCV processing)
                # Using 'bgr8' instead of 'bgra8' to avoid Alpha channel headaches
                original_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                
                # 2. Resize once at the start so coordinates match calibration
                frame = cv2.resize(original_frame, (self.image_width, self.image_height))
                
                # 3. Prepare detection frame (Gray) and drawing frame (Color copy)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                debug_frame = frame.copy() 
                
                if self.stop:
                    return

                corners, ids = None, None
                if version.parse(cv2.__version__) >= version.parse("4.7.0"):
                    corners, ids, _ = self.arucoDetector.detectMarkers(gray)
                else:
                    corners, ids, _ = cv2.aruco.detectMarkers(gray, self.arucoDict, parameters=self.arucoParams)

                if ids is not None and len(corners) > 0:
                    for markerCorner, markerID in zip(corners, ids.flatten()):
                        imagePoints = markerCorner.reshape((4, 2))
                        
                        success, rvec, tvec = cv2.solvePnP(
                            self.objectPoints,
                            imagePoints,
                            self.cameraMatrix,
                            self.distCoeffs
                        )
                        
                        if success:
                            # Pose logic
                            pose_msg = Pose()
                            pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = tvec.flatten()
                            
                            quaternion = quaternion_from_euler(rvec[0][0], rvec[1][0], rvec[2][0])
                            pose_msg.orientation.x = quaternion.x
                            pose_msg.orientation.y = quaternion.y
                            pose_msg.orientation.z = quaternion.z 
                            pose_msg.orientation.w = quaternion.w

                            self.pose_publisher.publish(pose_msg)
                            
                            # 4. Draw axes on the debug_frame
                            # Increased axis length to 0.1 for better visibility
                            cv2.drawFrameAxes(debug_frame, self.cameraMatrix, self.distCoeffs, rvec, tvec, 0.1)

                # 5. Always publish the debug_frame with explicit encoding
                # This prevents the "blank" screen by ensuring the msg header matches the data
                self.detected_publisher.publish(self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8"))

            except Exception as e:
                self.get_logger().error(f'Error processing frame: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()

    rclpy.spin(pose_publisher)

    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
