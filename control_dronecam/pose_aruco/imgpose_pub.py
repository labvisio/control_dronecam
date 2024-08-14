import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from message_filters import ApproximateTimeSynchronizer, SimpleFilter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from std_msgs.msg import Header

import numpy as np
import cv2
from cv_bridge import CvBridge


def intrinsic(data):
    K = np.reshape(data.p, (3, 4))
    intrinsic = np.delete(K, 3, 1)
    return intrinsic


def pose_img(frame, msg_calib, msg_gimbal, marker_length, arucoDict, parameters):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    K = intrinsic(data=msg_calib)
    D = np.array(msg_calib.d)
    angles_cam = None
    T = None
    Ry = np.array(
        [
            [np.cos(np.pi / 2), 0, np.sin(np.pi / 2), 0],
            [0, 1, 0, 0],
            [-np.sin(np.pi / 2), 0, np.cos(np.pi / 2), 0],
            [0, 0, 0, 1],
        ]
    )
    Rx = np.array(
        [
            [1, 0, 0, 0],
            [0, np.cos(-np.pi / 2), -np.sin(-np.pi / 2), 0],
            [0, np.sin(-np.pi / 2), np.cos(-np.pi / 2), 0],
            [0, 0, 0, 1],
        ]
    )
    corners, ids, _ = cv2.aruco.detectMarkers(
        image=gray,
        dictionary=arucoDict,
        parameters=parameters,
    )
    img_marked = cv2.aruco.drawDetectedMarkers(frame, corners)
    if ids is not None:
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners=corners,
            markerLength=marker_length,
            cameraMatrix=K,
            distCoeffs=D,
        )
        R_cam, _ = cv2.Rodrigues(rvec)
        tvec = tvec.reshape(3, 1)
        RT = np.hstack((R_cam, tvec))
        RT = np.vstack((RT, np.array([[0, 0, 0, 1]])))
        R_gimbal, _ = cv2.Rodrigues(
            np.array(
                [
                    np.radians(msg_gimbal.vector.x),
                    np.radians(msg_gimbal.vector.y),
                    0,
                ]
            )
        )
        R_gimbal = np.hstack((R_gimbal, np.array([[0], [0], [0]])))
        R_gimbal = np.vstack((R_gimbal, np.array([[0, 0, 0, 1]])))
        M = R_gimbal @ Rx @ Ry @ RT
        R = M[0:3, 0:3]
        T = M[:3, 3]
        _, _, _, _, _, _, angles_cam = cv2.decomposeProjectionMatrix(
            np.hstack((R, np.zeros((3, 1))))
        )
        angles_cam = np.radians(angles_cam.T)
        cv2.drawFrameAxes(
            image=img_marked,
            cameraMatrix=K,
            distCoeffs=D,
            rvec=rvec,
            tvec=tvec,
            length=10,
        )
    return img_marked, angles_cam, T


class Subscriber(SimpleFilter):

    def __init__(self, *args, **kwargs):
        SimpleFilter.__init__(self)
        self.node = args[0]
        self.topic = args[2]
        self.sub = self.node.create_subscription(
            *args[1:3], self.callback, qos_profile=args[3]
        )

    def callback(self, msg):
        self.signalMessage(msg)

    def getTopic(self):
        return self.topic

    def __getattr__(self, key):
        return self.sub.__getattribute__(key)


class Imageconsumer(Node):

    def parameter_callback(self, parameters):
        for param in parameters:
            if param.name == "aruco_dict":
                self.dict_aruco = param.value
                self.get_logger().info(
                    f"Updated type aruco dictionary {self.dict_aruco}"
                )
            elif param.name == "marker_length":
                self.marker_length = param.value
                self.get_logger().info(f"Updated lenght aruco {self.marker_length}")
            else:
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def __init__(self):
        super().__init__("imgcalib_pub")
        map_dictAruco = {
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

        # Parameter
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "aruco_dict",
                    "DICT_4X4_100",
                    ParameterDescriptor(description="Aruco dictionary"),
                ),
                (
                    "marker_length",
                    30,
                    ParameterDescriptor(description=" Size of the aruco"),
                ),
            ],
        )
        self.dict_aruco = map_dictAruco[self.get_parameter("aruco_dict").value]
        self.marker_length = self.get_parameter("marker_length").value
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Subscriber
        qos_profile_subscriber_best = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        qos_profile_subscriber_reliable = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE
        )
        image_consume = Subscriber(
            self,
            Image,
            "/anafi/camera/image",
            qos_profile_subscriber_best,
        )
        calib_consume = Subscriber(
            self,
            CameraInfo,
            "/anafi/camera/camera_info",
            qos_profile_subscriber_reliable,
        )
        gimbal_consume = Subscriber(
            self,
            Vector3Stamped,
            "anafi/gimbal/rpy",
            qos_profile_subscriber_best,
        )

        # Publisher
        self.pub_img = self.create_publisher(
            msg_type=Image,
            topic="/anafi/camera/image_aruco",
            qos_profile=10,
        )
        self.pose_aruco2Cam = self.create_publisher(
            msg_type=Odometry,
            topic="/anafi/pose/aruco2cam",
            qos_profile=10,
        )
        self.imageMsg = Image()
        self.odomMsg = Odometry()
        self.br = CvBridge()
        self.get_logger().info(
            "Created publisher of the image of the aruco and the position of the aruco in relation to the camera."
        )
        self.ts = ApproximateTimeSynchronizer(
            fs=[image_consume, calib_consume, gimbal_consume],
            queue_size=10,
            slop=0.1,
        )
        self.ts.registerCallback(self.callback)
        self.arucoDict = cv2.aruco.getPredefinedDictionary(self.dict_aruco)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.parameters)
        self.get_logger().info("Package initialized")

    def callback(self, msg_img, msg_calib, msg_gimbal):
        try:
            frame = self.br.imgmsg_to_cv2(msg_img, "bgr8")

            self.img_marked, self.R, self.T = pose_img(
                frame,
                msg_calib,
                msg_gimbal,
                self.marker_length,
                self.arucoDict,
                self.parameters,
            )
        except:
            self.get_logger().info("Frame lost")

    def publisher(self):
        self.imageMsg = self.br.cv2_to_imgmsg(self.img_marked, encoding="bgr8")
        self.pub_img.publish(self.imageMsg)

        if self.R is not None and self.T is not None:
            self.odomMsg.header = Header()
            self.odomMsg.header.stamp = self.get_clock().now().to_msg()
            self.odomMsg.header.frame_id = "/drone"
            self.odomMsg.pose.pose.position.x = self.T[0]
            self.odomMsg.pose.pose.position.y = self.T[1]
            self.odomMsg.pose.pose.position.z = self.T[2]
            self.odomMsg.pose.pose.orientation.x = self.R[0][0]
            self.odomMsg.pose.pose.orientation.y = self.R[0][1]
            self.odomMsg.pose.pose.orientation.z = self.R[0][2]
            print(self.odomMsg.pose.pose)
        else:
            self.get_logger().info("Aruco not detected.")
            self.odomMsg = Odometry()
        self.pose_aruco2Cam.publish(self.odomMsg)


def main():
    rclpy.init()
    image = Imageconsumer()
    rclpy.spin(image)
    image.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
