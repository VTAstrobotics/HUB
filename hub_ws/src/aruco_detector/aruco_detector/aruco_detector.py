#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_size = 0.072  # Marker size in meters (adjust based on your marker)

        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.info_callback, 10)

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/aruco_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('ArUco Detector Node Started')

    def info_callback(self, msg):
        # Store camera intrinsics and distortion coefficients
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info('Received camera info')

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn('Waiting for camera info...')
            return

        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Estimate pose for each detected marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == 64:  # Filter for marker ID 0
                    # Extract translation and rotation vectors
                    tvec = tvecs[i][0]  # Shape: (3,) -> [x, y, z]
                    rvec = rvecs[i][0]  # Shape: (3,) -> [rx, ry, rz]

                    # Convert rotation vector to rotation matrix
                    rot_mat, _ = cv2.Rodrigues(rvec)

                    # Create pose message
                    pose_msg = PoseStamped()
                    pose_msg.header = msg.header
                    pose_msg.header.frame_id = 'camera_frame'
                    pose_msg.pose.position.x = float(tvec[0])
                    pose_msg.pose.position.y = float(tvec[1])
                    pose_msg.pose.position.z = float(tvec[2])

                    # Convert rotation matrix to quaternion
                    w, x, y, z = self.rotation_matrix_to_quaternion(rot_mat)
                    pose_msg.pose.orientation.w = float(w)
                    pose_msg.pose.orientation.x = float(x)
                    pose_msg.pose.orientation.y = float(y)
                    pose_msg.pose.orientation.z = float(z)

                    # Publish pose
                    self.pose_pub.publish(pose_msg)
                    self.get_logger().info(f'Detected marker {marker_id} at position {tvec}')

                    # Publish visualization marker
                    marker_msg = Marker()
                    marker_msg.header = msg.header
                    marker_msg.header.frame_id = 'camera_frame'
                    marker_msg.type = Marker.CUBE
                    marker_msg.action = Marker.ADD
                    marker_msg.pose = pose_msg.pose
                    marker_msg.scale.x = self.marker_size
                    marker_msg.scale.y = self.marker_size
                    marker_msg.scale.z = 0.01
                    marker_msg.color.r = 1.0
                    marker_msg.color.g = 0.0
                    marker_msg.color.b = 0.0
                    marker_msg.color.a = 1.0
                    marker_msg.id = int(marker_id)
                    self.marker_pub.publish(marker_msg)

                    # Publish TF transform
                    t = TransformStamped()
                    t.header = msg.header
                    t.header.frame_id = 'camera_frame'
                    t.child_frame_id = 'aruco_frame'
                    t.transform.translation.x = float(tvec[0])
                    t.transform.translation.y = float(tvec[1])
                    t.transform.translation.z = float(tvec[2])
                    t.transform.rotation.w = float(w)
                    t.transform.rotation.x = float(x)
                    t.transform.rotation.y = float(y)
                    t.transform.rotation.z = float(z)
                    self.tf_broadcaster.sendTransform(t)

    def rotation_matrix_to_quaternion(self, R):
        # Convert rotation matrix to quaternion
        trace = np.trace(R)
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2.0
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
        return w, x, y, z

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()