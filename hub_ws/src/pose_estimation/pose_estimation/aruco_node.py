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

        self.pose_publisher = self.create_publisher(Pose, 'pose', 10)
        self.stop_subscription = self.create_subscription(
            Bool,
            'stop_aruco',
            self.stop_callback,
            10)
        
        self.image_subscription = self.create_subscription(
            Image,
            'image_raw',
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
        datapath = os.path.abspath(__file__).rsplit('/', 1)[0]
        paramPath = os.path.join(datapath, "matrixanddist.npz")
        print(paramPath)
        if not os.path.exists(paramPath):
            self.get_logger().error(".npz path does not exist")
            return
        data = np.load(paramPath)
        self.cameraMatrix = data['matrix']
        self.distCoeffs = data['distortion']
        
        self.markerLength = 0.1  # 10 cm tag size
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
        self.get_logger().info("Received image")


        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgra8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        # frame = self.bridge.imgmsg_to_cv2(msg, 'bgra8')



        if self.stop:
            return
        try:
            corners, ids = None, None
            frame = cv2.resize(frame, (800,600))
            frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY )
            if version.parse(cv2.__version__) >= version.parse("4.7.0"):
                corners, ids, _ = self.arucoDetector.detectMarkers(frame)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(frame, self.arucoDict, parameters = self.arucoParams)

            if ids is not None and len(corners) > 0:
                for markerCorner, markerID in zip(corners, ids.flatten()):
                    self.get_logger().info(f"Got ID {markerID}")
                    imagePoints = markerCorner.reshape((4, 2))
                    success, rvec, tvec = cv2.solvePnP(
                        self.objectPoints,
                        imagePoints,
                        self.cameraMatrix,
                        self.distCoeffs
                    )
                    
                    if success:

                        pose_msg = Pose()
                        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = tvec.flatten()
                        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z = rvec.flatten()[:3]
                        pose_msg.orientation.w = 0.0 
                        quaternion = quaternion_from_euler(rvec[0], rvec[1], rvec[2])
                        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z = quaternion.x, quaternion.y, quaternion.z 
                        pose_msg.orientation.w = quaternion.w

                        self.pose_publisher.publish(pose_msg)
                        self.get_logger().info(str(pose_msg))
                        self.get_logger().info(f"Published pose")
                        image_with_frame = self.bridge.imgmsg_to_cv2(msg)
                        
                        # cv2.drawFrameAxes(image_with_frame, self.cameraMatrix, self.distCoeffs, rvec, tvec, 0.05)
                        cv2.drawFrameAxes(image_with_frame, self.cameraMatrix, np.asarray(()), rvec, tvec, 0.05)
            else:
                self.get_logger().info("Nothing Detected in image")
                image_with_frame = None


        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')
        if image_with_frame is None:
            self.detected_publisher.publish(self.bridge.cv2_to_imgmsg(frame))
        else:
            self.detected_publisher.publish(self.bridge.cv2_to_imgmsg(image_with_frame))


def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()

    rclpy.spin(pose_publisher)

    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
