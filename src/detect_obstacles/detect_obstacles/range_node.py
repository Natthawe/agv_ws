import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time

class RangeObstacleDetectionNode(Node):

    def __init__(self):
        super().__init__('RANGE_OBSTACLE_DETECTION_NODE')
        self.node_name = "RANGE_OBSTACLE_DETECTION_NODE"
        self.get_logger().info(f'========={self.node_name}=========')
        
        # Set the desired obstacle detection range
        self.min_angle = -30.0  # Minimum angle (degrees)
        self.max_angle = 30.0   # Maximum angle (degrees)

        # Subscribe to the LaserScan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.obstacle_detection_callback,
            10  # Adjust the queue size as needed
        )

        # Publish Twist commands to control the robot
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize the Twist message to move forward
        self.twist_cmd = Twist()
        self.twist_cmd.linear.x = 0.5  # Set the linear speed (adjust as needed)

        # Variable to track the last time an obstacle was detected
        self.last_obstacle_time = time.time()        

    def obstacle_detection_callback(self, msg):
        # Convert LaserScan angles to radians
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
                
        # Filter the laser scan data within the desired range
        filtered_ranges = []
        filtered_angles = []
        for angle, distance in zip(angles, msg.ranges):
            if self.min_angle <= np.degrees(angle) <= self.max_angle:
                filtered_ranges.append(distance)
                filtered_angles.append(angle)

        # # Print the filtered distance and angle data
        # for angle, distance in zip(filtered_angles, filtered_ranges):
        #     self.get_logger().info(f"Angle: {np.degrees(angle):.2f} degrees, Distance: {distance:.2f} meters")

                # Check for obstacles within the desired range
        obstacle_detected = any(distance < 1.0 for distance in filtered_ranges)  # Adjust the threshold as needed

        # Stop or move forward based on obstacle detection
        if obstacle_detected:
            self.twist_cmd.linear.x = 0.0  # Set linear speed to 0 to stop
            self.last_obstacle_time = time.time()  # Update the time when an obstacle was last detected
        else:
            # Check if it's time to move forward again after the delay
            if time.time() - self.last_obstacle_time >= 2.0:
                self.twist_cmd.linear.x = 0.2  # Set the linear speed to move forward

        # Publish the Twist command
        self.publisher.publish(self.twist_cmd)        

def main(args=None):
    rclpy.init(args=args)
    obstacle_detection_node = RangeObstacleDetectionNode()
    rclpy.spin(obstacle_detection_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
