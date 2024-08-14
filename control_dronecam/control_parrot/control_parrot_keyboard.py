import rclpy
import numpy as np
from rclpy.node import Node
from anafi_autonomy.msg import KeyboardCommand
from std_msgs.msg import Header, Float32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class ParrotControl(Node):
    def __init__(self):
        super().__init__('parrot_control')

        qos_profile1 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        qos_profile2 = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
            depth=10
        )

        self.publisher_ = self.create_publisher(KeyboardCommand, '/anafi/keyboard/command', qos_profile2)
        self.subscription = self.create_subscription(Float32MultiArray, '/anafi/drone/position_global', self.position_callback, qos_profile1)
        self.timer = self.create_timer(0.1, self.publish_message)  # Publish at 10 Hz

        self.command = KeyboardCommand()
        self.command.header = Header()
        self.command.drone_action = 0
        self.command.drone_x = 0
        self.command.drone_y = 0
        self.command.drone_z = 0
        self.command.drone_yaw = 0
        self.command.gimbal_roll = 0
        self.command.gimbal_pitch = 0
        self.command.gimbal_yaw = 0
        self.command.camera_action = 0
        self.command.zoom = 0

        self.current_position = None

    def position_callback(self, msg):
        self.current_position = msg.data
        #self.get_logger().info(f'Received position: x={self.current_position[0]:.6f}m, y={self.current_position[1]:.6f}m, z={self.current_position[2]:.6f}m')

    def publish_message(self):
        if self.current_position:
            # Control logic here. For example, maintain a certain altitude or position.
            
            x = self.current_position[0]
            y = self.current_position[1]
            z = self.current_position[2]
            psi = self.current_position[3]
            Pos_VANT = np.array([x,y,z])

            try:
                g = g 
            except:
                g = 9.81
                tilt_max = (10*np.pi)/180
                psidot_max = (100*np.pi)/180
                
                U_inv = np.matrix([[(1/g*tilt_max), 0],
                                   [0, (1/g*tilt_max)]])
                zdot_max = 1
                kp = 1
                kp_psi = 1
                Pose_aruco = np.array([0,0,0])
                psi_aruco = 0
                Delta = np.array([-0.2, 0, 1])
            
            R = np.matrix([[np.cos(psi), np.sin(psi)],
                           [-np.sin(psi), np.cos(psi)]])
            
            #controle linear XY
            self.pos_desejada_VANT = Pose_aruco + Delta

            self.ac_ref = kp * (self.pos_desejada_VANT - Pos_VANT)
            self.ac_ref_xy = np.transpose(np.matrix([[self.ac_ref[0], self.ac_ref[1]]]))
            self.U = np.dot(np.dot(U_inv, R), self.ac_ref_xy)
            self.U = np.maximum(-1, np.minimum(1, self.U))

            #controle linar Z
            self.Zdot_ref = kp * (self.pos_desejada_VANT[2] - Pos_VANT[2])
            self.Zdot_ref = self.Zdot_ref/zdot_max
            self.Zdot_ref = max(-1, min(1, self.Zdot_ref))

            #correcao ref
            psi_erro = psi_aruco - psi
            if abs(psi_erro) > np.pi:
                if psi_erro > np.pi:
                    psi_erro = psi_erro - (2*np.pi)
                else:
                    psi_erro = psi_erro + (2*np.pi)

            #controle Yaw
            Yawdot_ref = kp_psi*(psi_erro)
            Yawdot_ref = Yawdot_ref/psidot_max
            Yawdot_ref = max(-1, min(1, Yawdot_ref))

            print('\n\nVel X',int(self.U[0]))
            print('Vel Y',self.U[1])
            print('Vel z',self.Zdot_ref)
            print('Vel Yaw',Yawdot_ref,'\n\n')

            self.command.header.stamp = self.get_clock().now().to_msg()
            print(type(self.U[0]))
            self.command.drone_x = float(self.U[0])
            self.command.drone_y = int(self.U[1])
            self.command.drone_z = int(self.Zdot_ref)
            self.command.drone_yaw = int(Yawdot_ref)
            
            # Print the received position data
            self.get_logger().info(f'Current position: {self.current_position}')

            self.publisher_.publish(self.command)
            self.get_logger().info(f'Publishing: {self.command}')

def main(args=None):
    rclpy.init(args=args)
    parrot_control = ParrotControl()
    rclpy.spin(parrot_control)
    parrot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
