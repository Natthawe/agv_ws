import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math

class OdometryCalculator(Node):

    def __init__(self):
        super().__init__('odometry_calculator')
        self.declare_parameter('wheel_radius', 0.05)  # in meters
        self.declare_parameter('wheelbase_width', 0.3)  # in meters
        self.declare_parameter('update_rate', 10)  # in Hz
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_time = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase_width = self.get_parameter('wheelbase_width').value
        self.update_rate = self.get_parameter('update_rate').value
        self.left_ticks_sub = self.create_subscription(Float64, 'left_wheel_ticks', self.left_ticks_callback, 10)
        self.right_ticks_sub = self.create_subscription(Float64, 'right_wheel_ticks', self.right_ticks_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)

    def left_ticks_callback(self, msg):
        current_left_ticks = msg.data
        if self.last_left_ticks != 0:
            left_ticks_diff = current_left_ticks - self.last_left_ticks
            right_ticks_diff = current_right_ticks - self.last_right_ticks
            self.last_left_ticks = current_left_ticks
            self.last_right_ticks = current_right_ticks
            self.update_odometry(left_ticks_diff, right_ticks_diff)

    def right_ticks_callback(self, msg):
        current_right_ticks = msg.data
        if self.last_right_ticks != 0:
            left_ticks_diff = current_left_ticks - self.last_left_ticks
            right_ticks_diff = current_right_ticks - self.last_right_ticks
            self.last_left_ticks = current_left_ticks
            self.last_right_ticks = current_right_ticks
            self.update_odometry(left_ticks_diff, right_ticks_diff)

    def update_odometry(self, left_ticks_diff, right_ticks_diff):
        # Calculate linear and angular displacement of the robot
        delta_left_distance = left_ticks_diff * self.wheel_radius
        delta_right_distance = right_ticks_diff * self.wheel_radius
        delta_distance = (delta_left_distance + delta_right_distance) / 2.0
        delta_theta = (delta_right_distance - delta_left_distance) / self.wheelbase_width
        # Update robot pose
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)
        self.theta += delta_theta
        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math
        odom_msg.twist.twist.linear.x = delta_distance / dt
        odom_msg.twist.twist.angular.z = delta_theta / dt
        self.odom_pub.publish(odom_msg)

    def timer_callback(self):
        # Send an initial odometry message
        if self.last_time is None:
            self.last_time = self.get_clock().now()
            odom_msg = Odometry()
            odom_msg.header.stamp = self.last_time.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    odometry_calculator = OdometryCalculator()
    rclpy.spin(odometry_calculator)
    odometry_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()