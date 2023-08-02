#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import time

class PIDController:
    def __init__(self, kp, ki, kd, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output

        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        if output > self.max_output:
            output = self.max_output
        elif output < -self.max_output:
            output = -self.max_output

        return output

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')

        # Subscribe to the camera image topic
        self.camera_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.line_detection_callback,
            10
        )

        self.image_pub = self.create_publisher(Image, "camera/cv_image", 10)

        # Publish Twist commands to control the robot
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize the Twist message to move forward
        self.twist_cmd = Twist()
        self.twist_cmd.linear.x = 0.5  # Set the linear speed (adjust as needed)   
        self.angular_speed = 0.0

        # Line detection variables
        self.middle = 0
        self.frame = None
        # PID controller for angular speed
        self.angular_pid = PIDController(kp=20.0, ki=0.0, kd=0.0, max_output=0.08)        

    def line_detection_callback(self, msg):

        bridge = CvBridge()
        self.frame = bridge.imgmsg_to_cv2(msg, "bgr8")

        try:
            frame_rgb = self.frame.copy()
        except:
            return

        hsv_image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        # Define the range for red color in HSV format
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for the two ranges of red color (since red hue wraps around 0-degree mark)
        red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        # Combine the masks to get the final red mask
        red_mask = red_mask1 + red_mask2

        # Use the mask to extract the red pixels from the original image
        red_filtered_image = cv2.bitwise_and(self.frame, self.frame, mask=red_mask)

        # convert black to white and white to black
        cv2.bitwise_not(red_mask, red_mask)

        diff = []
        points = []
        start_height = []
        self.middle = 0

        (h, w) = self.frame.shape[:2]
        sampling_points = 5
        sampling_gap = h // sampling_points


        for i in range(sampling_points):
            start_height.append(h - 1 - (sampling_gap * i))
            signed_thresh = red_mask[start_height[i]].astype(np.int16)  # select only one row
            diff.append(np.diff(signed_thresh))  # derivative

            points.append(np.where(np.logical_or(diff[i] > 200, diff[i] < -200)))  # maximums and minimums of derivative

            cv2.line(frame_rgb, (0, start_height[i]), (w, start_height[i]), (255, 0, 0), 1)

            # print(f'points: {points}')

            for j in range(len(points[i][0])):
                self.middle = points[i][0][j]
                cv2.circle(frame_rgb, (self.middle, start_height[i]), 5, (0, 0, 255), 1)
                cv2.circle(frame_rgb, (w // 2, start_height[i]), 5, (255, 0, 0), 1)
                cv2.line(frame_rgb, (self.middle, start_height[i]), (w // 2, start_height[i]), (0, 255, 0), 1)

        # Line following behavior
        if self.middle != 0:
            self.linear_speed = self.calculate_linear_speed(self.middle)                
            self.angular_speed = self.calculate_angular_speed(self.middle)             
        else:
            # If the line is lost, stop the robot
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            self.get_logger().info("Not found!!!")

        # Publish the velocity commands
        self.publish_velocity(self.linear_speed, self.angular_speed)

        red_mask_rgb = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2RGB)
        merge = np.hstack((self.frame, frame_rgb))
        merge2 = np.hstack((red_mask_rgb, red_filtered_image))
        merge = np.vstack((merge, merge2))

        processed_image_msg = bridge.cv2_to_imgmsg(frame_rgb, "bgr8")
        self.image_pub.publish(processed_image_msg)

    def calculate_linear_speed(self, middle):
        # max_linear_speed = 0.5  # Maximum linear speed (adjust as needed)
        # distance_from_center = abs(middle - (self.frame.shape[1] // 2))
        # linear_speed = max_linear_speed * (1 - distance_from_center / (self.frame.shape[1] // 2))
        linear_speed = 0.5
        return linear_speed

    # turn right -
    # turn left +

    def calculate_angular_speed(self, pp):
        err = (pp - 80)*(-1)
        val = err / 800
        return val

    def publish_velocity(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.publisher.publish(twist_msg) 

def main(args=None):
    rclpy.init(args=args)
    image_processing_node = ImageProcessingNode()

    try:
        rclpy.spin(image_processing_node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    image_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()