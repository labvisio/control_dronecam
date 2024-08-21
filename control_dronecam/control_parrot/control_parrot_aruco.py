import rclpy
import numpy as np
from rclpy.node import Node
from anafi_ros_interfaces.msg import PilotingCommand
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
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

        #Define limites de controle
        self.lim_horiontal_vel = 0.5 #M/s
        self.lim_vertical_vel = 0.2 #M/s
        self.lim_horizontal_tilt = 3 #Maximo de inclinação
        self.Delta = np.array([0, 0, 1]) #Define o Delta que deve ser mantido do ArUco

        self.first_sample = None
        self.current_yaw = None
        
        #Create subscriptions and publisher
        self.publisher_ = self.create_publisher(PilotingCommand, '/anafi/drone/command', qos_profile2)
        self.subscription = self.create_subscription(Vector3Stamped, '/anafi/drone/speed', self.speed_callback, qos_profile1)
        self.subscription = self.create_subscription(Odometry, '/anafi/pose/aruco2cam', self.aruco_pose_callback, qos_profile1)
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
        self.aruco_pose = None

        print("subscribed in all topics")

    def speed_callback(self, msg):
        self.current_speed = (msg.vector.x, msg.vector.y, msg.vector.z)
    
    def aruco_pose_callback(self, msg):
        self.aruco_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.z)

    def publish_message(self):
        if self.aruco_pose is not None:

            x = -self.aruco_pose[0]/100
            y = -self.aruco_pose[1]/100
            z = -self.aruco_pose[2]/100
            psi = -self.aruco_pose[3]
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
                self.psi_aruco = 0

            #trajetoria/pose
            Pose_aruco = np.array([0,0,0])

            #controle linear XY
            self.pos_desejada_VANT = Pose_aruco + self.Delta

            self.vel_ref = self.kp * (self.pos_desejada_VANT - Pos_VANT)
            self.vel_ref = np.transpose(np.matrix([[self.vel_ref[0], self.vel_ref[1]]])) #transpor e descartar Z
            self.vel_ref_R = np.maximum(-0.5, np.minimum(0.5, self.vel_ref))
            
            #controle de aceleracao
            self.ac_ref = self.kd*(self.vel_ref_R - drone_vel)
            self.U = np.dot(self.U_inv, self.ac_ref)
            self.U = np.maximum(-self.lim_horizontal_tilt, np.minimum(self.lim_horizontal_tilt, self.U))
            
            #controle linar Z
            self.Zdot_ref = self.kp * (self.pos_desejada_VANT[2] - Pos_VANT[2])
            self.Zdot_ref = self.Zdot_ref/self.zdot_max
            self.Zdot_ref = max(-self.lim_vertical_vel, min(self.lim_vertical_vel, self.Zdot_ref))

            #correcao ref YAW (Sempre o drone vai corrigir orientar pelo caminho mais curto)
            psi_erro =  psi - self.psi_aruco
            if abs(psi_erro) > np.pi:
                if psi_erro > np.pi:
                    psi_erro = psi_erro - (2*np.pi)
                else:
                    psi_erro = psi_erro + (2*np.pi)

            #controle Yaw
            Yawdot_ref = self.kp_psi*(psi_erro)
            Yawdot_ref = Yawdot_ref*(180/np.pi)

            #print para aferir o sinal de controle
            print('\n\n\n\n\n\n\n\n\n\n\n\nRoll',round(float(-self.U[1]), 4),'Pitch',round(float(self.U[0]), 4))
            print('Vel_ref_X',round(float(self.vel_ref_R[0]), 4),'Vel_ref_Y',round(float(self.vel_ref_R[1]), 4))
            print('Yawdot_ref',round(float(Yawdot_ref), 4),'Zdot_ref',round(float(self.Zdot_ref), 4))
            
            #Não publica caso o ArUco não seja detectado (Deixando livre o /Drone/Command)
            if (self.aruco_pose[0] + self.aruco_pose[1] + self.aruco_pose[2]) != 0: 
                self.command.header.stamp = self.get_clock().now().to_msg()
                self.command.roll = float(-self.U[1,0])
                self.command.pitch = float(self.U[0,0])
                self.command.gaz = float(self.Zdot_ref)
                self.command.yaw = float(-Yawdot_ref)    
                self.publisher_.publish(self.command)

            else:
                pass
            
def main(args=None):
    rclpy.init(args=args)
    parrot_control = ParrotControl()
    rclpy.spin(parrot_control)
    parrot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
