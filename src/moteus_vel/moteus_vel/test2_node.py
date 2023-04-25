import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.subscription = self.create_subscription(
            Int32,
            'wheel_encoder_feedback',
            self.feedback_callback,
            10)
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.prev_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def feedback_callback(self, msg):
        curr_time = self.get_clock().now()
        dt = (curr_time - self.prev_time).nanoseconds / 1e9

        # calculate distance traveled and change in orientation
        distance = msg.data * wheel_radius
        delta_theta = (distance / wheel_base) * math.tan(delta_steering)

        # update position and orientation
        self.x += distance * math.cos(self.theta + delta_theta / 2)
        self.y += distance * math.sin(self.theta + delta_theta / 2)
        self.theta += delta_theta

        # create and publish odometry message
        odom = Odometry()
        odom.header.stamp = curr_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        quat = Quaternion()
        quat.z = math.sin(self.theta / 2)
        quat.w = math.cos(self.theta / 2)
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = distance / dt
        odom.twist.twist.angular.z = delta_theta / dt
        self.publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
