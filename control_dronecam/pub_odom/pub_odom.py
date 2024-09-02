import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32, Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np


def euler_to_quaternion(roll, pitch, yaw):
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    return (qx, qy, qz, qw)


class OdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("odom_publisher")
        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = None
        self.initial_yaw_received = False
        self.previous_time = self.get_clock().now()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.setup_subscribers()
        self.setup_publishers()

    def setup_subscribers(self) -> None:
        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Vector3Stamped,
            "/anafi/drone/speed",
            self.velocity_callback,
            qos_profile,
        )
        self.create_subscription(
            Float32,
            "/anafi/drone/altitude",
            self.altitude_callback,
            qos_profile,
        )
        self.create_subscription(
            Vector3Stamped,
            "/anafi/drone/rpy",
            self.rpy_callback,
            qos_profile,
        )
        self.create_timer(
            timer_period_sec=0.1,
            callback=self.publish_odometry_message,
        )

    def setup_publishers(self) -> None:
        self.odom_publisher = self.create_publisher(
            Odometry,
            "/anafi/drone/odom",
            qos_profile=10,
        )

    def velocity_callback(self, msg: Vector3Stamped) -> None:
        self.vel_drone_x = msg.vector.x
        self.vel_drone_y = msg.vector.y
        self.vel_drone_z = msg.vector.z

    def altitude_callback(self, msg: Float32) -> None:
        self.altitude = msg.data

    def rpy_callback(self, msg: Vector3Stamped) -> None:
        self.roll = np.radians(msg.vector.x)
        self.pitch = np.radians(msg.vector.y)
        if not self.initial_yaw_received:
            self.initial_yaw = msg.vector.z
            self.initial_yaw_received = True
        else:
            self.yaw = np.radians(msg.vector.z - self.initial_yaw)

    def publish_odometry_message(self) -> None:
        if self.yaw is None:
            return
        self.current_time = self.get_clock().now()
        self.dt = (self.current_time - self.previous_time).nanoseconds / 1e9
        self.previous_time = self.current_time
        self.update_position()
        self.publish_pose()
        self.publish_tf()

    def update_position(self) -> None:
        velocity_vector = np.array([[self.vel_drone_x], [self.vel_drone_y]])
        rotation_matrix = np.array(
            [
                [np.cos(self.yaw), -np.sin(self.yaw)],
                [np.sin(self.yaw), np.cos(self.yaw)],
            ]
        )
        global_velocity = rotation_matrix @ velocity_vector
        self.quaternion = euler_to_quaternion(
            self.roll,
            self.pitch,
            self.yaw,
        )
        self.position_x += global_velocity[0][0] * self.dt
        self.position_y += global_velocity[1][0] * self.dt

    def publish_pose(self) -> None:
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.position.z = self.altitude
        odom_msg.pose.pose.orientation.x = self.quaternion[0]
        odom_msg.pose.pose.orientation.y = self.quaternion[1]
        odom_msg.pose.pose.orientation.z = self.quaternion[2]
        odom_msg.pose.pose.orientation.w = self.quaternion[3]
        odom_msg.twist.twist.linear.x = self.vel_drone_x
        odom_msg.twist.twist.linear.y = self.vel_drone_y
        odom_msg.twist.twist.angular.z = self.vel_drone_z
        self.odom_publisher.publish(odom_msg)

    def publish_tf(self) -> None:
        tf = TransformStamped()
        tf.header = Header()
        tf.header.stamp = self.current_time.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.position_x
        tf.transform.translation.y = self.position_y
        tf.transform.translation.z = self.altitude
        tf.transform.rotation.x = self.quaternion[0]
        tf.transform.rotation.y = self.quaternion[1]
        tf.transform.rotation.z = self.quaternion[2]
        tf.transform.rotation.w = self.quaternion[3]
        self.tf_broadcaster.sendTransform(tf)


def main() -> None:
    rclpy.init()
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
