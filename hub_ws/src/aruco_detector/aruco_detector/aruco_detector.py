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
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException
import tf_transformations



# TODO modify so paramaterize camera topic and name


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_size = 0.072  # Marker size in meters (adjust based on your marker)

        self.declare_parameter(
            "camera_name",
            "camera_front",
            ParameterDescriptor(description="camera name to listen to"),
        )

        self.camera_name = (
            self.get_parameter("camera_name").get_parameter_value().string_value
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, "/" + self.camera_name + "/image_raw", self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, "/" + self.camera_name + "/camera_info", self.info_callback, 10
        )

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, "/aruco_pose", 10)
        self.marker_pub = self.create_publisher(Marker, "/aruco_marker", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.robot_pose_pub = self.create_publisher(
            PoseStamped, f"/aruco_pose_{self.camera_name}", 10
        )

        self.get_logger().info("ArUco Detector Node Started")

    def info_callback(self, msg):
        # Store camera intrinsics and distortion coefficients
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("Received camera info")

    
    def transform_to_matrix(self, ts):
        translation = [ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z]
        rotation = [ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w]

        mat = tf_transformations.concatenate_matrices(
            tf_transformations.translation_matrix(translation),
            tf_transformations.quaternion_matrix(rotation)
        )
        return mat

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn("Waiting for camera info...")
            return

        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None:
            # Estimate pose for each detected marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

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
                    pose_msg.header.frame_id = self.camera_name + "_frame"
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
                    self.get_logger().info(
                        f"Detected marker {marker_id} at position {tvec}"
                    )

                    # Publish visualization marker
                    marker_msg = Marker()
                    marker_msg.header = msg.header
                    marker_msg.header.frame_id = self.camera_name + "_frame"
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
                    # do all transform math here
                    t = TransformStamped()
                    t.header = msg.header
                    t.header.frame_id = self.camera_name + "_frame"
                    t.child_frame_id = f'aruco_tag_{marker_id}'
                    t.transform.translation.x = float(tvec[0])
                    t.transform.translation.y = float(tvec[1])
                    t.transform.translation.z = float(tvec[2])
                    t.transform.rotation.w = float(w)
                    t.transform.rotation.x = float(x)
                    t.transform.rotation.y = float(y)
                    t.transform.rotation.z = float(z)
                    # self.tf_broadcaster.sendTransform(t)

                    # get absolute position of detected tag
                    tag_map = self._tf_buffer.lookup_transform(
                        f"static_tag_{marker_id}", "map", rclpy.time.Time()
                    )
                    camera_base_link = self._tf_buffer.lookup_transform(
                       self.camera_name + "_link",  "base_link", rclpy.time.Time()
                    )
                    # need camera  -> base link transform as well

                    tag_from_map_matrix = self.transform_to_matrix(tag_map) #drone math.
                    camera_from_base_link_matrix = self.transform_to_matrix(camera_base_link)

                    camera_tag_matrix = self.transform_to_matrix(t)
                    tag_camera_matrix = np.linalg.inv(camera_tag_matrix)

                    base_link_matrix = np.dot(np.dot(tag_from_map_matrix, tag_camera_matrix), camera_from_base_link_matrix)

                    translation = tf_transformations.translation_from_matrix(base_link_matrix)
                    quaternion = tf_transformations.quaternion_from_matrix(base_link_matrix)

                    pose_msg = PoseStamped()
                    pose_msg.header = msg.header
                    pose_msg.header.frame_id = "base_link"
                    pose_msg.pose.position.x = translation.x
                    pose_msg.pose.position.y = translation.y
                    pose_msg.pose.position.z = translation.z

                    pose_msg.pose.orientation.w = quaternion.w
                    pose_msg.pose.orientation.x = quaternion.x
                    pose_msg.pose.orientation.y = quaternion.y
                    pose_msg.pose.orientation.z = quaternion.z

                    # Publish pose
                    self.pose_pub.publish(pose_msg)
                    self.get_logger().info(f"Published backcomputed pose")

                    pose_msg = PoseStamped()
                    pose_msg.header = msg.header
                    pose_msg.header.frame_id = self.camera_name + "_frame"
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
                    self.robot_pose_pub.publish(pose_msg)
                    self.get_logger().info(
                        f"Detected marker {marker_id} at position {tvec}"
                    )

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


if __name__ == "__main__":
    main()
