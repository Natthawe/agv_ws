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

            # Create a publisher to publish odometry messages
        self.pub_odom_wheel = self.create_publisher(Odometry, "odometry/wheel", 10)
        # subscribe to moteusStates
        self.create_subscription(MoteusStateStamped, 'moteus_feedback', self.feedback_callback, 1)

    def feedback_callback(self, msg):
        # Implement your code to process the feedback message here
        # For example, you can calculate the odometry based on the received MoteusStateStamped message
        odometry_msg = Odometry()
        odometry_msg.header = msg.header
        odometry_msg.child_frame_id = "base_link"
        odometry_msg.pose.pose.position.x = msg.state[0].position
        odometry_msg.twist.twist.linear.x = msg.state[0].velocity

        # Publish the odometry message
        self.pub_odom_wheel.publish(odometry_msg)
        

def main():
    rclpy.init()
    moteus_control = MoteusControlNode()
    rclpy.spin(moteus_control)
    moteus_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()