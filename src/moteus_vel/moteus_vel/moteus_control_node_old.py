#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from moteus_msgs.msg import MoteusCommand, MoteusCommandStamped, MoteusStateStamped, MoteusState
from math import sin, cos, pi

class MoteusControlNode(Node):
    def __init__(self):
        super().__init__('MOTEUS_VEL_NODE')
        self.nodename = "MOTEUS_VEL_NODE"
        self.get_logger().info(f"{self.nodename} STARTED")

        ### variable init ###
        self.x = 0.0  # position in xy plane
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0  # speeds in x/rotation
        self.dr = 0.0
        self.then = self.get_clock().now()
        self.states = None
        self.time_state = self.get_clock().now().to_msg()
        self.recv_state = {}
        
        self.wheel_left = self.wheel_right = 0.0
        
        #### init global value ####
        self.enc_wheel_left = 0
        self.enc_wheel_right = 0
        self.enc_wheel_left_pv = 0
        self.enc_wheel_right_pv = 0
        
        ### enc funtion wrap value init ###
        self.enc_wheel_left_mult = 0.0
        self.enc_wheel_right_mult = 0.0
        self.prev_enc_left = 0
        self.prev_enc_right = 0

        self.NS_TO_SEC = 1000000

        #### frame id node ####
        self.base_frame_id = self.declare_parameter('base_frame_id','base_link').value
        self.odom_frame_id = self.declare_parameter('odom_frame_id','odom').value
        
        #### encoder min max value ####
        self.encoder_min = self.declare_parameter('encoder_min', -2147483648).value
        self.encoder_max = self.declare_parameter('encoder_max', 2147483647).value

        #### encoder min max wrap value ####
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (
                self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (
                self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value
        
        #### parameters ####
        self.ticks_meter = float(self.declare_parameter('ticks_meter', 34278.0).value) 
        self.base_width = float(self.declare_parameter('base_width', 0.65).value)  
        self.radius_of_wheels = float(self.declare_parameter('radius_of_wheels', 0.254).value)  

        #publish moteus_command
        self.pub = self.create_publisher(MoteusCommandStamped, 'moteus_command', 10)
        timer_period = 0.01 # seconds
        self.timer = self.create_timer(timer_period, self.callback_update)       
        
        # subscribe to cmd_vel
        self.create_subscription(Twist, 'scurve_cmd_vel', self.cmd_vel_callback, 10)  
        
        # subscribe to moteusStates
        # self.create_subscription(MoteusStateStamped, 'moteus_feedback', self.callback_command, 10)   


    def cmd_vel_callback(self, twist):
      
        V = twist.linear.x
        W = twist.angular.z

        self.wheel_left = (V - (W * self.base_width / 2)) / self.radius_of_wheels
        self.wheel_right = (V + (W * self.base_width / 2)) / self.radius_of_wheels
        # print(wheel_left, wheel_right)
        messages = str(self.wheel_left) + "," + str(self.wheel_right) + '\n'
     
    def callback_update(self):
        moteusCommandStamped = MoteusCommandStamped()
        moteusCommandStamped.header.stamp = self.get_clock().now().to_msg()
        moteusCommandStamped.header.frame_id = "moteus_command"

        moteusCommand = MoteusCommand()
        moteusCommand.device_id = 1
        moteusCommand.velocity = self.wheel_left * 30.0
        moteusCommand.maximum_torque = 1.7
        moteusCommandStamped.commands.append(moteusCommand)

        moteusCommand = MoteusCommand()
        moteusCommand.device_id = 2
        moteusCommand.velocity = self.wheel_right * 30.0
        moteusCommand.maximum_torque = 1.7
        moteusCommandStamped.commands.append(moteusCommand)

        self.pub.publish(moteusCommandStamped) 

def main():
    rclpy.init()
    moteus_control = MoteusControlNode()
    rclpy.spin(moteus_control)
    moteus_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()