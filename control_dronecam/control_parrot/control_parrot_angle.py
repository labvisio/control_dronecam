import rclpy
import numpy as np
from inputs import get_gamepad
from rclpy.node import Node
from anafi_ros_interfaces.msg import PilotingCommand
from geometry_msgs.msg import Vector3Stamped
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

        self.publisher_ = self.create_publisher(PilotingCommand, '/anafi/drone/command', qos_profile2)
        #self.publisher_vel = self.create_publisher(Vector3Stamped, '/anafi/drone/speed', qos_profile1)
        self.subscription = self.create_subscription(Float32MultiArray, '/anafi/drone/position_global', self.position_callback, qos_profile1)
        self.timer = self.create_timer(0.001, self.publish_message)  # Publish at 1000 Hz

        self.command = PilotingCommand()
        self.command.header = Header()
        self.command.roll = float(0)
        self.command.pitch = float(0)
        self.command.yaw = float(0)
        self.command.gaz = float(0)
        print('done')
        self.current_position = None

        self.first_sample = None

    def position_callback(self, msg):
        self.current_position = msg.data
        #self.get_logger().info(f'Received position: x={self.current_position[0]:.6f}m, y={self.current_position[1]:.6f}m, z={self.current_position[2]:.6f}m')

    def publish_message(self):
        if self.current_position:
            x = self.current_position[0]
            y = self.current_position[1]
            z = self.current_position[2]
            psi = self.current_position[3]
            Pos_VANT = np.array([x,y,z])
            #print(Pos_VANT)

            try:
                g = g 
            except:
                g = 9.81
                tilt_max = 10
                U_inv = np.matrix([[(1/g*tilt_max), 0],
                                   [0, (1/g*tilt_max)]])
                zdot_max = 1
                kp = 2
                kp_psi = 1
                Pose_aruco = np.array([0,0,0])
                psi_aruco = 0
                Delta = np.array([0, 0, 1])
            
            R = np.matrix([[np.cos(psi), np.sin(psi)],
                           [-np.sin(psi), np.cos(psi)]])
            
            #controle linear XY
            self.pos_desejada_VANT = Pose_aruco + Delta

            self.ac_ref = kp * (self.pos_desejada_VANT - Pos_VANT)
            self.ac_ref_xy = np.transpose(np.matrix([[self.ac_ref[0], self.ac_ref[1]]]))
            self.U = np.dot(np.dot(U_inv, R), self.ac_ref_xy)
            self.U = np.maximum(-5, np.minimum(5, self.U))
            
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
            Yawdot_ref = max(-1, min(1, Yawdot_ref))
            Yawdot_ref = Yawdot_ref*(180/np.pi)

            print('\n\nRoll',float(self.U[1]),'Pitch',float(-self.U[0]))

            self.command.header.stamp = self.get_clock().now().to_msg()
            self.command.roll = float(self.U[1,0])
            self.command.pitch = float(-self.U[0,0])
            self.command.gaz = float(self.Zdot_ref)
            self.command.yaw = float(Yawdot_ref)

            self.publisher_.publish(self.command)

def main(args=None):
    rclpy.init(args=args)
    parrot_control = ParrotControl()
    rclpy.spin(parrot_control)
    parrot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
