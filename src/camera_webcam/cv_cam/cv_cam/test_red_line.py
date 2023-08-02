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
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize the Twist message to move forward
        self.twist_cmd = Twist()
        self.twist_cmd.linear.x = 0.5      

        # Line detection variables
        self.RLine = 0
        self.point_count = 0
        self.middle = 0 

        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.1
        self.angular_speed = 0.0
        self.linear_speed = 0.0
        self.linear_speed_reduction_factor = 1.0
        self.previous_angular_speed = 0.0    

        self.frame_count = 0
        self.start_time = time.time()
        self.pid_controller = PIDController(kp=1.0, ki=0.0, kd=0.0, setpoint=80)            

    def line_detection_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        try:
            frame_rgb = frame.copy()
        except:
            return

        # Convert the image to the HSV color space for better color segmentation
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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

        # Find contours in the red mask
        contours, _ = cv2.findContours(red_mask, 1, cv2.CHAIN_APPROX_NONE)
        # cv2.drawContours(frame_rgb, contours, -1, (0, 255, 0), 3)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Draw a rectangle around the red line area
                cv2.rectangle(frame_rgb, (x, y), (x + w, y + h), (255, 255, 0), 2)   

                # Draw the red line on the original image
                for contour in contours:
                    if cv2.contourArea(contour) > 100:  # Minimum area to consider as a line (you may need to adjust this value)
                        x, y, w, h = cv2.boundingRect(contour)
                        cv2.line(frame_rgb, (x + (w // 2), y), (x + (w // 2), y + h), (0, 0, 255), 3)


                # self.get_logger().info("CX: %d  CY: %d" % (cx, cy))
                output = self.pid_controller.update(cx)
                # self.get_logger().info("PID Output: %.2f" % output)

                linear = self.calculate_linear_speed(cx)
                angular = self.calculate_angular_speed(output, cx)

                self.publish_velocity(linear, angular)                         
                
            else:
                self.get_logger().info("No valid moments found.")
                self.publish_velocity(0.0, 0.0)  # Stop the robot if no valid moments found
        else:
            self.get_logger().info("No contours found.")
            self.publish_velocity(0.0, 0.0)  # Stop the robot if no contours found

        # Publish the processed image (in RGB format) to a new topic for visualization (if required)
        processed_image_msg = bridge.cv2_to_imgmsg(frame_rgb, "bgr8")
        self.image_pub.publish(processed_image_msg)   

        self.frame_count += 1
        if self.frame_count % 1 == 0:  # Calculate framerate every 1 frames
            elapsed_time = time.time() - self.start_time
            framerate = self.frame_count / elapsed_time
            self.get_logger().info("Framerate: %.2f fps" % framerate)             

    def calculate_linear_speed(self, angular_speed):
        if self.previous_angular_speed != 0.0 and angular_speed != self.previous_angular_speed:
            linear = self.max_linear_speed * self.linear_speed_reduction_factor
        else:
            linear = self.max_linear_speed

        if linear > self.max_linear_speed:
            linear = self.max_linear_speed

        self.previous_angular_speed = angular_speed
        return linear


    def calculate_angular_speed(self, output, cx):
        image_width = 160
        period_width = image_width // 5

        if period_width * 2 <= cx < period_width * 3:
            return 0.0  # Run straight in the middle period

        if 0 <= cx < period_width * 2:  # Range is inclined to the left
            angle_percentage = (period_width * 2 - cx) / period_width

            # Gradually increase angular speed until reaching the full value
            angular_speed = self.max_angular_speed * angle_percentage
            return angular_speed

        if period_width * 3 <= cx < period_width * 5:  # Range is inclined to the right
            angle_percentage = (cx - period_width * 3) / period_width

            # Gradually increase angular speed until reaching the full value
            angular_speed = -self.max_angular_speed * angle_percentage
            return angular_speed

        return 0.0  # Default case: Run straight

    def publish_velocity(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.cmd_vel_pub.publish(twist_msg)

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
