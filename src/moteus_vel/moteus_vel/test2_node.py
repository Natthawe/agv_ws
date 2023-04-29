#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from moteus_drive.MoteusDrive import MoteusDrive
from moteus_msgs.msg import MoteusCommand, MoteusCommandStamped, MoteusStateStamped, MoteusState
from math import sin, cos, pi
import moteus
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class MoteusControlNode(Node):
    def __init__(self):
        super().__init__('MOTEUS_VEL_NODE')
        self.nodename = "MOTEUS_VEL_NODE"
        self.get_logger().info(f"{self.nodename} STARTED")

        self.wheel_left = 0.0
        self.wheel_right = 0.0        

        # Internal Data Variable
        # self.x = 0.0 # position in xy plane
        # self.y = 0.0
        # self.th = 0.0
        # self.dx = 0.0 # speeds in x/rotation
        # self.dr = 0.0
        # self.then = self.get_clock().now() # time for determining dx/dy
        # self.enc_wheel_left = 0.0
        # self.enc_wheel_right = 0.0
        # self.enc_wheel_left_pv = 0
        # self.enc_wheel_right_pv = 0  
        # self.enc_wheel_left_mult = 0.0
        # self.enc_wheel_right_mult = 0.0              
        # self.prev_enc_left = 0
        # self.prev_enc_right = 0
        # self.ns_to_sec = 1.0e-9

        self.last_time = self.get_clock().now()
        self.last_pos_left = 0
        self.last_pos_right = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.vx = 0
        self.vy = 0
        self.vtheta = 0
        self.wheel_diameter = 0.127 # in meters
        self.gear_ratio = 30.0
        self.ns_to_sec = 1.0e-9


        # Parameters
        self.ticks_meter = float(self.declare_parameter('ticks_meter', 31.5195990935).value) #2*pi*0.127*39.5*30
        self.base_width = float(self.declare_parameter('base_width', 0.65).value)  
        self.radius_of_wheels = float(self.declare_parameter('radius_of_wheels', 0.254/2).value) 
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.encoder_min = self.declare_parameter('encoder_min', -2147483648).value
        self.encoder_max = self.declare_parameter('encoder_max', 2147483647).value
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (
            self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (
            self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value

        # publisher
        self.pub_odom_wheel = self.create_publisher(Odometry, "odometry/wheel", 10)
        self.pub = self.create_publisher(MoteusCommandStamped, 'moteus_command', 1)

        # subscribe
        self.create_subscription(MoteusStateStamped, 'moteus_feedback', self.feedback_callback, 1)
        self.create_subscription(Twist, 'cmd_vel_accel_decel', self.cmd_vel_callback, 1)  #scurve_cmd_vel       
        

    def feedback_callback(self, msg):

        time = self.get_clock().now()
        dt = (time - self.last_time).nanoseconds / self.ns_to_sec

        pos_left = msg.state[0].position
        pos_right = msg.state[0].position

        d_left = (pos_left - self.last_pos_left) * self.wheel_diameter / self.gear_ratio
        d_right = (pos_right - self.last_pos_right) * self.wheel_diameter / self.gear_ratio

        delta_s = (d_left + d_right) / 2.0
        delta_theta = (d_right - d_left) / self.base_width

        self.vx = delta_s / dt
        self.vtheta = delta_theta / dt

        self.x += delta_s * cos(self.theta)
        self.y += delta_s * sin(self.theta)
        self.theta += delta_theta

        odom_quat = Quaternion()
        odom_quat.z = sin(self.theta / 2.0)
        odom_quat.w = cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame 
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = odom_quat
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vtheta

        self.pub_odom_wheel.publish(odom)

        self.last_pos_left = pos_left
        self.last_pos_right = pos_right
        self.last_time = time        

    def cmd_vel_callback(self, twist):

        V = twist.linear.x
        W = twist.angular.z

        self.wheel_left = (V + (W * self.base_width / 2)) / self.radius_of_wheels
        self.wheel_right = (V - (W * self.base_width / 2)) / self.radius_of_wheels

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

def main():
    rclpy.init()
    moteus_control = MoteusControlNode()
    rclpy.spin(moteus_control)
    moteus_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()