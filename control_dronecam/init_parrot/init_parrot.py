import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from anafi_ros_interfaces.msg import GimbalCommand
from std_msgs.msg import Header


class DroneInit(Node):
    def __init__(self):
        super().__init__("drone_init")
        self.takeoff_client = self.create_service_client("/anafi/drone/takeoff")
        self.land_client = self.create_service_client("/anafi/drone/land")
        self.angle_gimbal = GimbalCommand()
        self.setup_publish()
        self.publish_msg()

    def setup_publish(self):
        self.angle_gimbal_pub = self.create_publisher(
            msg_type=GimbalCommand,
            topic="anafi/gimbal/command",
            qos_profile=10,
        )

    def publish_msg(self):
        self.angle_gimbal.header = Header()
        self.angle_gimbal.header.stamp = self.get_clock().now().to_msg()
        self.angle_gimbal.header.frame_id = "camera"
        self.angle_gimbal.mode = 0
        self.angle_gimbal.frame = 1
        self.angle_gimbal.roll = 0.0
        self.angle_gimbal.pitch = 90.0
        self.angle_gimbal.yaw = 0.0
        self.angle_gimbal_pub.publish(self.angle_gimbal)
        self.get_logger().info(f"Publish angle gimbal")

    def create_service_client(self, service_name: str):
        client = self.create_client(Trigger, service_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {service_name} service...")
        return client

    def send_request(self, client, action: str):
        request = Trigger.Request()
        future = client.call_async(request)
        self.get_logger().info(f"{action.capitalize()} request sent.")
        return future

    def send_takeoff_request(self):
        return self.send_request(self.takeoff_client, "takeoff")

    def send_land_request(self):
        return self.send_request(self.land_client, "landing")


def main():
    rclpy.init()
    drone_client = DroneInit()
    drone_client.send_takeoff_request()
    try:
        rclpy.spin(drone_client)
    except KeyboardInterrupt:
        drone_client.send_land_request()
        drone_client.get_logger().info("Ctrl+C detected, initiating landing...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
