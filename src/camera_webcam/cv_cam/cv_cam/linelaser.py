import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import time
import math


class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.error = 0.0
        self.last_error = 0.0
        self.error_sum = 0.0
        self.last_time = time.time()

    def update(self, current_value):
        current_time = time.time()
        dt = current_time - self.last_time

        self.error = self.setpoint - current_value
        self.error_sum += self.error * dt
        error_diff = (self.error - self.last_error) / dt

        output = self.kp * self.error + self.ki * self.error_sum + self.kd * error_diff

        self.last_error = self.error
        self.last_time = current_time

        return output


class LineAndObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('line_and_obstacle_detection_node')
        
        # Set the desired obstacle detection range
        self.min_angle = -60.0  # Minimum angle (degrees)
        self.max_angle = 60.0   # Maximum angle (degrees)

        # Subscribe to the LaserScan topic
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.obstacle_detection_callback,
            10  # Adjust the queue size as needed
        )

        # Subscribe to the camera topic
        self.image_pub = self.create_publisher(Image, "camera/cv_image", 10)
        self.camera_sub = self.create_subscription(
            Image, "camera/image_raw", self.line_detection_callback, 10
        )

        # Publish Twist commands to control the robot
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.frame_count = 0
        self.start_time = time.time()
        self.pid_controller = PIDController(kp=5.0, ki=0.0, kd=0.0, setpoint=80)
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.1

        self.linear_speed_reduction_factor = 1.0  # Adjust the reduction factor as needed
        self.previous_angular_speed = 0.0

        self.RLine = 0
        self.point_count = 0
        self.last_point_count = 0
        self.count = 0
        self.middle = 0
        self.mark = 0
        
        self.last_obstacle_time = time.time()

    def obstacle_detection_callback(self, msg):
        # Filter the laser scan data within the desired range
        filtered_ranges = []
        filtered_angles = []
        for angle, distance in zip(msg.angle_min + msg.angle_increment * range(len(msg.ranges)), msg.ranges):
            if self.min_angle <= angle <= self.max_angle:
                filtered_ranges.append(distance)
                filtered_angles.append(angle)

        # Check for obstacles within the desired range
        obstacle_detected = any(distance < 0.5 for distance in filtered_ranges)  # Adjust the threshold as needed

        # Stop or move forward based on obstacle detection
        if obstacle_detected:
            self.twist_cmd.linear.x = 0.0  # Set linear speed to 0 to stop
            self.last_obstacle_time = time.time()  # Update the time when an obstacle was last detected
        else:
            # Check if it's time to move forward again after the delay
            if time.time() - self.last_obstacle_time >= 2.0:
                self.twist_cmd.linear.x = 0.2  # Set the linear speed to move forward

        # Publish the Twist command
        self.cmd_vel_pub.publish(self.twist_cmd)

    def line_detection_callback(self, msg):
        # Convert the ROS2 Image message to OpenCV format
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Line detection logic
        # ... (same as before)

        if self.middle == 0:
            self.get_logger().info("Not found!!!")
            self.twist_cmd.linear.x = 0.0  # Set linear speed to 0 to stop
        else:   
            angular_speed = self.calculate_angular_speed(self.middle)
            linear_speed = self.calculate_linear_speed(angular_speed)
            self.twist_cmd.linear.x = linear_speed
            self.twist_cmd.angular.z = angular_speed

        # Publish the Twist command
        self.cmd_vel_pub.publish(self.twist_cmd)

        img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(img_msg)

    # ... (same as before for calculate_linear_speed and calculate_angular_speed methods)

def main(args=None):
    rclpy.init(args=args)
    node = LineAndObstacleDetectionNode()
    rclpy.spin(node)
    cv2.destroyAllWindows() 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
