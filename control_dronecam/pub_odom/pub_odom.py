import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np


class OdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("odom_publisher")
        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = None
        self.initial_yaw_received = False
        self.previous_time = self.get_clock().now()
        self.setup_subscribers()
        self.setup_publishers()
        self.setup_timer()

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

    def setup_publishers(self) -> None:
        self.odom_publisher = self.create_publisher(
            Odometry,
            "/anafi/drone/odom",
            qos_profile=10,
        )

    def setup_timer(self) -> None:
        self.create_timer(
            timer_period_sec=0.1,
            callback=self.publish_odometry_message,
        )

    def velocity_callback(self, msg: Vector3Stamped) -> None:
        self.vel_drone_x = msg.vector.x
        self.vel_drone_y = msg.vector.y
        self.vel_drone_z = msg.vector.z

    def altitude_callback(self, msg: Float32) -> None:
        self.altitude = msg.data

    def rpy_callback(self, msg: Vector3Stamped) -> None:
        self.rpy = msg.vector
        if not self.initial_yaw_received:
            self.initial_yaw = self.rpy.z
            self.initial_yaw_received = True
        else:
            self.yaw = np.radians(self.rpy.z - self.initial_yaw)

    def publish_odometry_message(self) -> None:
        if self.yaw is None:
            return
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9
        self.update_position(dt)
        self.previous_time = current_time
        self.publish_pose(current_time)

    def update_position(self, dt: float) -> None:
        velocity_vector = np.array([[self.vel_drone_x], [self.vel_drone_y]])
        rotation_matrix = np.array(
            [
                [np.cos(self.yaw), -np.sin(self.yaw)],
                [np.sin(self.yaw), np.cos(self.yaw)],
            ]
        )
        global_velocity = rotation_matrix @ velocity_vector
        self.position_x += global_velocity[0][0] * dt
        self.position_y += global_velocity[1][0] * dt

    def publish_pose(self, current_time) -> None:
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.position.z = self.altitude
        odom_msg.pose.pose.orientation.x = self.rpy.x
        odom_msg.pose.pose.orientation.y = self.rpy.y
        odom_msg.pose.pose.orientation.z = self.yaw
        odom_msg.pose.pose.orientation.w = 1.0
        odom_msg.twist.twist.linear.x = self.vel_drone_x
        odom_msg.twist.twist.linear.y = self.vel_drone_y
        odom_msg.twist.twist.angular.z = self.vel_drone_z
        self.odom_publisher.publish(odom_msg)


def main() -> None:
    rclpy.init()
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
