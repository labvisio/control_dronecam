import rclpy
import math
import numpy as np
from rclpy.node import Node
from anafi_ros_interfaces.msg import PilotingCommand
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header, Float32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import time
import keyboard

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
        self.subscription = self.create_subscription(Vector3Stamped, '/anafi/drone/speed', self.speed_callback, qos_profile1)
        self.timer = self.create_timer(0.001, self.publish_message)  # Publish at 1000 Hz

        self.command = PilotingCommand()
        self.command.header = Header()
        self.command.roll = float(0)
        self.command.pitch = float(0)
        self.command.yaw = float(0)
        self.command.gaz = float(0)
        self.current_position = None
        self.current_speed = (0.0, 0.0, 0.0)

        self.first_time = 1

    def position_callback(self, msg):
        self.current_position = msg.data
        #self.get_logger().info(f'Received position: x={self.current_position[0]:.6f}m, y={self.current_position[1]:.6f}m, z={self.current_position[2]:.6f}m')

    def speed_callback(self, msg):
        self.current_speed = (msg.vector.x, msg.vector.y, msg.vector.z)
        print(self.current_speed,'Drone_vel')

    def publish_message(self):
        if self.current_position:
            #posicao
            x = self.current_position[0]
            y = self.current_position[1]
            z = self.current_position[2]
            psi = self.current_position[3]
            Pos_VANT = np.array([x,y,z])
            
            #velocidade
            dot_x = self.current_speed[0]
            dot_y = self.current_speed[1]
            drone_vel = np.transpose(np.matrix([[dot_x, dot_y]]))

            if self.first_time == 1:
                self.first_time = 0
                self.g = 9.81
                self.tilt_max = 15
                self.U_inv = np.matrix([[(1/self.g*self.tilt_max), 0],
                                   [0, (1/self.g*self.tilt_max)]])
                self.zdot_max = 1
                self.kp = 1
                self.kd = 3
                self.kp_psi = 1
                self.Pose_aruco_old = Pos_VANT
                self.psi_aruco = 0
                self.Delta = np.array([0, 0, 1])
                #trajetoria
                self.r = 1
                self.w = (2*np.pi)/100


            t = time.time()

            #trajetoria/pose
            #Pose_aruco = np.array([0,0,0])
            Pose_aruco = np.array([self.r*np.sin(self.w*t),self.r*np.sin(2*self.w*t),0]) #lemniscata
            #Pose_aruco = np.array([r*np.cos(w*t),r*np.sin(w*t),0]) #circulo
            
            #orientacao
            #psi_aruco = np.arctan2(Pose_aruco[1] - Pose_aruco_old[1], Pose_aruco[0] - Pose_aruco_old[0])
            #Pose_aruco_old = Pos_VANT
            #print(psi_aruco)
            
            R = np.matrix([[np.cos(psi), np.sin(psi)],
                           [-np.sin(psi), np.cos(psi)]])
            
            #controle linear XY
            self.pos_desejada_VANT = Pose_aruco + self.Delta

            self.vel_ref = self.kp * (self.pos_desejada_VANT - Pos_VANT)
            self.vel_ref = np.transpose(np.matrix([[self.vel_ref[0], self.vel_ref[1]]])) #transpor e descartar Z
            self.vel_ref_R = np.dot(R, self.vel_ref) #velocidade de referencia
            self.vel_ref_R = np.maximum(-2, np.minimum(2, self.vel_ref_R))
            
            #controle de aceleracao
            self.ac_ref = self.kd*(self.vel_ref_R - drone_vel)
            self.U = np.dot(self.U_inv, self.ac_ref)
            self.U = np.maximum(-10, np.minimum(10, self.U))
            
            #controle linar Z
            self.Zdot_ref = self.kp * (self.pos_desejada_VANT[2] - Pos_VANT[2])
            self.Zdot_ref = self.Zdot_ref/self.zdot_max
            self.Zdot_ref = max(-1, min(1, self.Zdot_ref))

            #correcao ref
            psi_erro = self.psi_aruco - psi
            if abs(psi_erro) > np.pi:
                if psi_erro > np.pi:
                    psi_erro = psi_erro - (2*np.pi)
                else:
                    psi_erro = psi_erro + (2*np.pi)

            #controle Yaw
            Yawdot_ref = self.kp_psi*(psi_erro)
            Yawdot_ref = Yawdot_ref*(180/np.pi)

            #print(Yawdot_ref)
            print('\n\nRoll',round(float(-self.U[1]), 4),'Pitch',round(float(self.U[0]), 4))
            print('Vel_ref_X',round(float(self.vel_ref_R[0]), 4),'Vel_ref_Y',round(float(self.vel_ref_R[1]), 4))
            self.command.header.stamp = self.get_clock().now().to_msg()
            self.command.roll = float(-self.U[1,0])
            self.command.pitch = float(self.U[0,0])
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
