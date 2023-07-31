#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import time
import math

# rad = np.arange(12.)
# degrees = np.degrees(rad)
# print(degrees)

class LaserRange(Node):
    def __init__(self):
        super().__init__('LaserRange')

        # Set the desired obstacle detection range
        self.min_angle = -30.0  # Minimum angle (degrees)
        self.max_angle = 30.0   # Maximum angle (degrees)

        # Subscribe to the LaserScan topic
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_range_callback,
            10  # Adjust the queue size as needed
        )

        # Publish Twist commands to control the robot
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)        

    def laser_range_callback(self, msg):
        # Convert LaserScan angles to radians
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        # Filter the laser scan data within the desired range
        filtered_ranges = []
        for angle, distance in zip(angles, msg.ranges):
            if self.min_angle <= np.degrees(angle) <= self.max_angle:
                filtered_ranges.append(distance)

        # Check for obstacles within the desired range
        self.obstacle_detected = any(distance < 0.65 for distance in filtered_ranges)  # Adjust the threshold as needed

        if self.obstacle_detected:
            self.stop_time = time.time()
            self.obstacle_detected = True
            self.publish_velocity(0.0, 0.0)
        else:
            # If no obstacle is detected, check the time difference before allowing the robot to move forward again
            if self.obstacle_detected and time.time() - self.stop_time >= 2.0:
                self.obstacle_detected = False

    def publish_velocity(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserRange()
    rclpy.spin(node)
    cv2.destroyAllWindows() 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()