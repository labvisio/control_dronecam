import rclpy
from rclpy.node import Node
from anafi_autonomy.msg import KeyboardCommand
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import time

class KeyboardCommandPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_command_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
            depth=10
        )
        
        self.publisher_ = self.create_publisher(KeyboardCommand, '/anafi/keyboard/command', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_message)  # Publish at 10 Hz

        self.command = KeyboardCommand()
        self.command.header = Header()
        self.command.drone_action = 0
        self.command.drone_x = 0.1
        self.command.drone_y = 0
        self.command.drone_z = 0
        self.command.drone_yaw = 1
        self.command.gimbal_roll = 0
        self.command.gimbal_pitch = 0
        self.command.gimbal_yaw = 0
        self.command.camera_action = 0
        self.command.zoom = 0

    def publish_message(self):
        self.command.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.command)
        self.get_logger().info(f'Publishing: {self.command}')

def main(args=None):
    rclpy.init(args=args)
    keyboard_command_publisher = KeyboardCommandPublisher()
    rclpy.spin(keyboard_command_publisher)
    keyboard_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
