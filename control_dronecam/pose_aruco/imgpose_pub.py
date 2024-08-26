import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from message_filters import ApproximateTimeSynchronizer, SimpleFilter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

import numpy as np
import cv2
from cv_bridge import CvBridge
from typing import Tuple, Optional


def extract_intrinsics(camera_info: CameraInfo) -> np.ndarray:
    """Extracts the intrinsic camera matrix from CameraInfo."""
    K = np.reshape(camera_info.p, (3, 4))
    intrinsic_matrix = np.delete(K, 3, 1)
    return intrinsic_matrix


def compute_pose_from_image(
    frame: np.ndarray,
    camera_info: CameraInfo,
    gimbal_data: Vector3Stamped,
    marker_length: float,
    aruco_dict: cv2.aruco.Dictionary,
    aruco_id: float,
    detector_params: cv2.aruco.DetectorParameters,
) -> Tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
    """Computes pose from the detected ArUco markers in the image."""
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    K = extract_intrinsics(camera_info)
    D = np.array(camera_info.d)
    R_yaw_90 = np.array(
        [
            [np.cos(np.pi / 2), 0, np.sin(np.pi / 2), 0],
            [0, 1, 0, 0],
            [-np.sin(np.pi / 2), 0, np.cos(np.pi / 2), 0],
            [0, 0, 0, 1],
        ]
    )
    R_pitch_neg_90 = np.array(
        [
            [1, 0, 0, 0],
            [0, np.cos(-np.pi / 2), -np.sin(-np.pi / 2), 0],
            [0, np.sin(-np.pi / 2), np.cos(-np.pi / 2), 0],
            [0, 0, 0, 1],
        ]
    )
    corners, ids, _ = cv2.aruco.detectMarkers(
        image=gray_image,
        dictionary=aruco_dict,
        parameters=detector_params,
    )
    img_marked = cv2.aruco.drawDetectedMarkers(frame, corners)
    if ids is not None:
        for id, corner in zip(ids, corners):
            if id == aruco_id:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners=corner,
                    markerLength=marker_length,
                    cameraMatrix=K,
                    distCoeffs=D,
                )
                rotation_camera, _ = cv2.Rodrigues(rvec)
                tvec = tvec.reshape(3, 1)
                camera_to_marker = np.hstack((rotation_camera, tvec))
                camera_to_marker = np.vstack(
                    (camera_to_marker, np.array([[0, 0, 0, 1]]))
                )
                rotation_gimbal, _ = cv2.Rodrigues(
                    np.radians(
                        [
                            gimbal_data.vector.x,
                            gimbal_data.vector.y,
                            0,
                        ]
                    )
                )
                gimbal_to_drone = np.hstack((rotation_gimbal, np.zeros((3, 1))))
                gimbal_to_drone = np.vstack((gimbal_to_drone, np.array([[0, 0, 0, 1]])))
                combined_transformation = (
                    gimbal_to_drone @ R_pitch_neg_90 @ R_yaw_90 @ camera_to_marker
                )
                rotation_matrix = combined_transformation[:3, :3]
                translation_vector = combined_transformation[:3, 3]
                _, _, _, _, _, _, euler_angles = cv2.decomposeProjectionMatrix(
                    np.hstack((rotation_matrix, np.zeros((3, 1))))
                )
                euler_angles = np.radians(euler_angles.T)
                cv2.drawFrameAxes(
                    image=img_marked,
                    cameraMatrix=K,
                    distCoeffs=D,
                    rvec=rvec,
                    tvec=tvec,
                    length=marker_length,
                )
                return img_marked, euler_angles, translation_vector
    return img_marked, None, None


class ROS2Subscriber(SimpleFilter):
    def __init__(self, node: Node, msg_type, topic_name: str, qos_profile: QoSProfile):
        super().__init__()
        self.node = node
        self.topic_name = topic_name
        self.subscription = self.node.create_subscription(
            msg_type, topic_name, self.callback, qos_profile
        )

    def callback(self, msg):
        self.signalMessage(msg)

    def get_topic_name(self) -> str:
        return self.topic_name


class ImageProcessor(Node):
    def __init__(self):
        super().__init__("image_processor")
        self.declare_ros_parameters()
        self.arucoDict = cv2.aruco.getPredefinedDictionary(self.aruco_dict)
        self.aruco_detector_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(
            dictionary=self.arucoDict,
            detectorParams=self.aruco_detector_params,
        )
        self.setup_subscribers()
        self.setup_publishers()
        self.br = CvBridge()
        self.msg_gimbal_ok = False

    def declare_ros_parameters(self):
        aruco_dict_mapping = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        }
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "aruco_dict",
                    rclpy.Parameter.Type.STRING,
                    ParameterDescriptor(description="Aruco dictionary"),
                ),
                (
                    "marker_lenght",
                    rclpy.Parameter.Type.INTEGER,
                    ParameterDescriptor(description="Size of the ArUco marker"),
                ),
                (
                    "aruco_id",
                    rclpy.Parameter.Type.INTEGER,
                    ParameterDescriptor(description="Id aruco"),
                ),
            ],
        )
        self.aruco_dict = aruco_dict_mapping[self.get_parameter("aruco_dict").value]
        self.marker_length = self.get_parameter("marker_lenght").value
        self.aruco_id = self.get_parameter("aruco_id").value
        self.add_on_set_parameters_callback(self.on_parameter_update)

    def on_parameter_update(self, parameters):
        for param in parameters:
            if param.name == "aruco_dict":
                self.aruco_dict = param.value
                self.get_logger().info(f"Updated ArUco dictionary to {self.aruco_dict}")
            elif param.name == "marker_length":
                self.marker_length = param.value
                self.get_logger().info(f"Updated marker length to {self.marker_length}")
            elif param.name == "aruco_id":
                self.aruco_id = param.value
                self.get_logger().info(f"Update ArUco id to {self.aruco_id}")
            else:
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def setup_subscribers(self):
        qos_profile_best_effort = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        qos_profile_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.image_sub = self.create_subscription(
            msg_type=Image,
            topic="/anafi/camera/image",
            callback=self.image_sub_callback,
            qos_profile=qos_profile_best_effort,
        )
        self.camera_info_sub = self.create_subscription(
            msg_type=CameraInfo,
            topic="/anafi/camera/camera_info",
            callback=self.image_info_callback,
            qos_profile=qos_profile_reliable,
        )
        self.gimbal_info_sub = self.create_subscription(
            msg_type=Vector3Stamped,
            topic="/anafi/gimbal/rpy_slow/relative",
            callback=self.gimbal_info_callback,
            qos_profile=qos_profile_best_effort,
        )
        self.timer = self.create_timer(
            timer_period_sec=0.033,
            callback=self.publish_message,
        )

    def setup_publishers(self):
        self.image_pub = self.create_publisher(
            msg_type=Image,
            topic="/anafi/camera/image_aruco",
            qos_profile=10,
        )
        self.pose_pub = self.create_publisher(
            msg_type=Odometry,
            topic="/anafi/pose/aruco2cam",
            qos_profile=10,
        )

    def image_sub_callback(self, image):
        self.frame = self.br.imgmsg_to_cv2(image, "bgr8")

    def image_info_callback(self, info_camera):
        self.camera_info_msg = info_camera

    def gimbal_info_callback(self, rpy_gimbal):
        if not self.msg_gimbal_ok:
            self.msg_gimbal_ok = True
            self.get_logger().info("Image Processor Node initialized.")
        self.gimbal_msg = rpy_gimbal

    def publish_message(self):
        if self.msg_gimbal_ok:
            try:
                marked_frame, rotation, translation = compute_pose_from_image(
                    frame=self.frame,
                    camera_info=self.camera_info_msg,
                    gimbal_data=self.gimbal_msg,
                    marker_length=self.marker_length,
                    aruco_dict=self.arucoDict,
                    aruco_id=self.aruco_id,
                    detector_params=self.aruco_detector_params,
                )
                self.publish_pose(translation=translation, rotation=rotation)
                self.publish_image(image=marked_frame)
            except:
                self.get_logger().info("frame not processed")
                pass
        else:
            pass

    def publish_image(self, image: np.ndarray):
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera"
        img_msg = self.br.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_pub.publish(img_msg)

    def publish_pose(self, translation: np.ndarray, rotation: np.ndarray):
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "drone"
        if rotation is not None and translation is not None:
            odom_msg.pose.pose.position.x = translation[0]
            odom_msg.pose.pose.position.y = translation[1]
            odom_msg.pose.pose.position.z = translation[2]
            odom_msg.pose.pose.orientation.x = rotation[0][0]
            odom_msg.pose.pose.orientation.y = rotation[0][1]
            odom_msg.pose.pose.orientation.z = rotation[0][2]
        self.pose_pub.publish(odom_msg)


def main():
    rclpy.init()
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
