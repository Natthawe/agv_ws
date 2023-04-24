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


        
        self.wheel_left = self.wheel_right = 0.0
         
        self.base_width = float(self.declare_parameter('base_width', 0.65).value)  
        self.radius_of_wheels = float(self.declare_parameter('radius_of_wheels', 0.254/2).value)  

        #publish moteus_command
        self.pub = self.create_publisher(MoteusCommandStamped, 'moteus_command', 1)
        # timer_period = 0.01 # seconds
        # self.timer = self.create_timer(timer_period, self.callback_update)       
        
        # subscribe to cmd_vel
        self.create_subscription(Twist, 'cmd_vel_accel_desel', self.cmd_vel_callback, 1)  #scurve_cmd_vel
        
        # subscribe to moteusStates
        # self.create_subscription(MoteusStateStamped, 'moteus_feedback', self.callback_command, 10)   


    def cmd_vel_callback(self, twist):
      
        V = twist.linear.x
        W = twist.angular.z

        self.wheel_left = (V + (W * self.base_width / 2)) / self.radius_of_wheels
        self.wheel_right = (V - (W * self.base_width / 2)) / self.radius_of_wheels
        # print(wheel_left, wheel_right)
        # messages = str(self.wheel_left) + "," + str(self.wheel_right) + '\n'
        moteusCommandStamped = MoteusCommandStamped()
        moteusCommandStamped.header.stamp = self.get_clock().now().to_msg()
        moteusCommandStamped.header.frame_id = "moteus_command"

        moteusCommand = MoteusCommand()
        moteusCommand.device_id = 1
        moteusCommand.velocity = self.wheel_left * -4.5
        moteusCommand.maximum_torque = 1.7
        moteusCommandStamped.commands.append(moteusCommand)

        moteusCommand = MoteusCommand()
        moteusCommand.device_id = 2
        moteusCommand.velocity = self.wheel_right * -4.5
        moteusCommand.maximum_torque = 1.7
        moteusCommandStamped.commands.append(moteusCommand)

        self.pub.publish(moteusCommandStamped) 
     
    def callback_update(self):
        pass

def main():
    rclpy.init()
    moteus_control = MoteusControlNode()
    rclpy.spin(moteus_control)
    moteus_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()