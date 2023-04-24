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

        # initialize variables and objects
        self.states = None
        self.time_feedback = self.get_clock().now().to_msg()
        self.time_feedback_seconds = self.get_clock().now().to_msg()
        self.recv_feedback = {}        

        self.wheel_left = 0.0
        self.wheel_right = 0.0        

        # Internal Data Variable
        self.x = 0.0 # position in xy plane
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0 # speeds in x/rotation
        self.dr = 0.0
        self.then = self.get_clock().now() # time for determining dx/dy
        self.enc_wheel_left = 0.0
        self.enc_wheel_right = 0.0
        self.enc_wheel_left_pv = 0
        self.enc_wheel_right_pv = 0  
        self.enc_wheel_left_mult = 0.0
        self.enc_wheel_right_mult = 0.0              
        self.prev_enc_left = 0
        self.prev_enc_right = 0
        self.ns_to_sec = 1.0e-9


        # Parameters
        self.ticks_meter = float(self.declare_parameter('ticks_meter', 1170.0).value)
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

        #publish moteus_command
        self.pub = self.create_publisher(MoteusCommandStamped, 'moteus_command', 1)

        # Publish Topic odom_wheel
        self.pub_odom_wheel = self.create_publisher(Odometry, "odometry/wheel", 10)

        # TF BroadCaster
        self.odom_broadcaster = TransformBroadcaster(self)

        # RATE TIMER HZ
        self.rate = self.declare_parameter("rate", 100.0).value
        self.create_timer(1.0/self.rate, self.update)  
        
        # subscribe to cmd_vel
        self.create_subscription(Twist, 'cmd_vel_accel_decel', self.cmd_vel_callback, 1)  #scurve_cmd_vel
        
        # subscribe to moteusStates
        self.create_subscription(MoteusStateStamped, 'moteus_feedback', self.callback_feedback, 1)   


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

    def callback_feedback(self, msg):
        self.time_feedback = msg.header.stamp
        self.state = msg.state
        self.recv_feedback = {}
        for idx, states in enumerate(self.state):
            # self.get_logger().info(f"{idx}")
            self.recv_feedback[states.device_id] = {"mode": states.mode, "position": states.position, "velocity": states.velocity}
        # send commands
        # self.get_logger().info(f"{self.recv_feedback[2]}")
        # # self.get_logger().info(f"{idx}")
        self.enc_wheel_left_raw = self.recv_feedback[1]["position"]
        self.enc_wheel_right_raw = self.recv_feedback[2]["position"]
        # self.get_logger().info(f"{self.enc_wheel_left_raw , enc_wheel_right_raw}")
        
        if self.enc_wheel_left_raw < self.encoder_low_wrap and self.prev_enc_left > self.encoder_high_wrap:
            self.enc_wheel_left_mult = self.enc_wheel_left_mult + 1
            
        if self.enc_wheel_left_raw > self.encoder_high_wrap and self.prev_enc_left < self.encoder_low_wrap:
            self.enc_wheel_left_mult = self.enc_wheel_left_mult - 1
            
        self.enc_wheel_left = 1.0 * (self.enc_wheel_left_raw + self.enc_wheel_left_mult * (self.encoder_max - self.encoder_min))
        self.prev_enc_left = self.enc_wheel_left_raw
        
        if self.enc_wheel_right_raw < self.encoder_low_wrap and self.prev_enc_right > self.encoder_high_wrap:
            self.enc_wheel_right_mult = self.enc_wheel_right_mult + 1
            
        if self.enc_wheel_right_raw > self.encoder_high_wrap and self.prev_enc_right < self.encoder_low_wrap:
            self.enc_wheel_right_mult = self.enc_wheel_right_mult - 1
            
        self.enc_wheel_right = 1.0 * (self.enc_wheel_right_raw + self.enc_wheel_right_mult * (self.encoder_max - self.encoder_min))
        self.prev_enc_right = self.enc_wheel_right_raw
        # self.get_logger().info(f"{enc_wheel_left , enc_wheel_right}")
        
        return self.enc_wheel_left, self.enc_wheel_right          
        

    def update(self, msg):
        
        self.enc_wheel_left, self.enc_wheel_right = self.callback_feedback(msg)
        # Calculate Time
        now = self.get_clock().now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.nanoseconds / self.ns_to_sec

        # Calculate Odometry
        distance_wheel_left = (self.enc_wheel_left - self.enc_wheel_left_pv) / self.ticks_meter
        distance_wheel_right = (self.enc_wheel_right - self.enc_wheel_right_pv) / self.ticks_meter        

        # Distance traveled is the average of the two wheels 
        dist = (distance_wheel_left + distance_wheel_right) / 2

        # This approximation works (in radians) for small angles
        th = (distance_wheel_left - distance_wheel_right) / self.base_width

        self.dx = dist / elapsed
        self.dr = th / elapsed

        if dist != 0:
            # Calculate Distance Traveled in x and y
            x = cos(th) * dist
            y = -sin(th) * dist
            # Calculate the final position of the robot
            self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
            self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
        if th != 0:
            self.th = self.th + th   

        # Publish Info odom_wheel
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = now.to_msg()
        transform_stamped_msg.header.frame_id = self.odom_frame
        transform_stamped_msg.child_frame_id = self.base_frame
        transform_stamped_msg.transform.translation.x = self.x
        transform_stamped_msg.transform.translation.y = self.y
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.w = quaternion.w
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z

        # self.odom_broadcaster.sendTransform(transform_stamped_msg)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.dr
        self.pub_odom_wheel.publish(odom)

        self.enc_wheel_left_pv = self.enc_wheel_left
        self.enc_wheel_right_pv = self.enc_wheel_right        

def main():
    rclpy.init()
    moteus_control = MoteusControlNode()
    rclpy.spin(moteus_control)
    moteus_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()