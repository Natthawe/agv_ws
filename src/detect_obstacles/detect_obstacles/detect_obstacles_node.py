import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class DetectObstacles(Node):
    def __init__(self):
        super().__init__('DetectObstacles')
        self.nodename = "DETECT_OBSTACLES"
        self.get_logger().info(f"========={self.nodename}=========")

        self.laser_subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',  # Update this topic if your LiDAR publishes on a different topic
            self.laser_callback,
            10
        )

        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize the movement command
        self.cmd_vel_ = Twist()
        self.cmd_vel_.linear.x = 0.2  # Adjust the linear speed as needed
        self.cmd_vel_.angular.z = 0.0

        # Set the number of frontal segments
        self.num_segments_ = 3  # Adjust the number of segments for the frontal 180-degree range
        self.segment_angle_ = 180.0 / self.num_segments_

        # Initialize the obstacle flags for each segment
        self.obstacles_in_segment_ = [False] * self.num_segments_

        # Flag to track the obstacle state
        self.obstacle_detected_ = False

    def laser_callback(self, msg):
        num_ranges = len(msg.ranges)
        # print(num_ranges)

        # Process the laser scan data to check for obstacles in each segment
        for i in range(self.num_segments_):
            # Calculate the start and end index of the current segment in the laser scan data
            start_idx = math.floor((i * self.segment_angle_) / msg.angle_increment)
            end_idx = math.floor(((i + 1) * self.segment_angle_) / msg.angle_increment)

            # Make sure the indices are within the valid range
            start_idx = min(max(start_idx, 0), num_ranges - 1)
            end_idx = min(max(end_idx, 180), num_ranges - 1)

            # Check for obstacles in the current segment
            self.obstacles_in_segment_[i] = False
            for idx in range(start_idx, end_idx):
                if msg.ranges[idx] < 0.5:  # Set an appropriate threshold distance here
                    self.obstacles_in_segment_[i] = True
                    break

        # Generate and publish the movement command based on obstacle detection
        any_obstacle = any(self.obstacles_in_segment_)

        if any_obstacle:
            self.cmd_vel_.linear.x = 0.0  # Stop moving
            self.obstacle_detected_ = True
        else:
            if self.obstacle_detected_:
                # Delay for 2 seconds
                time.sleep(2)
                self.obstacle_detected_ = False

            self.cmd_vel_.linear.x = 0.2  # Keep moving forward

        self.cmd_vel_publisher_.publish(self.cmd_vel_)


def main(args=None):
    rclpy.init(args=args)
    controller = DetectObstacles()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
