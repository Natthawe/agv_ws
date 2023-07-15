#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import time


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


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.image_pub = self.create_publisher(Image, "camera/cv_image", 10)
        self.camera_sub = self.create_subscription(
            Image, "camera/image_raw", self.callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.frame_count = 0
        self.start_time = time.time()
        self.pid_controller = PIDController(kp=0.1, ki=0.0, kd=0.05, setpoint=400)
        self.max_linear_speed = 1.2
        self.max_angular_speed = 0.2

        self.linear_speed_reduction_factor = 1.0  # Adjust the reduction factor as needed
        self.previous_angular_speed = 0.0

    def callback(self, msg):
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")

        low_b = np.uint8([100, 100, 100])
        high_b = np.uint8([0, 0, 0])
        mask = cv2.inRange(frame, high_b, low_b)
        contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 10)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                # self.get_logger().info("CX: %d  CY: %d" % (cx, cy))
                output = self.pid_controller.update(cx)
                self.get_logger().info("PID Output: %.2f" % output)

                linear = self.calculate_linear_speed(output)
                angular = self.calculate_angular_speed(cx)

                self.publish_velocity(linear, angular)
                cv2.circle(frame, (cx, cy), 10, (255, 255, 255), -1)
            else:
                self.get_logger().info("No valid moments found.")
                self.publish_velocity(0.0, 0.0)  # Stop the robot if no valid moments found
        else:
            self.get_logger().info("No contours found.")
            self.publish_velocity(0.0, 0.0)  # Stop the robot if no contours found

        # To display the image, convert it to ROS2 format and publish
        img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(img_msg)

        self.frame_count += 1
        if self.frame_count % 10 == 0:  # Calculate framerate every 10 frames
            elapsed_time = time.time() - self.start_time
            framerate = self.frame_count / elapsed_time
            # self.get_logger().info("Framerate: %.2f fps" % framerate)

    def calculate_linear_speed(self, angular_speed):
        if self.previous_angular_speed != 0.0 and angular_speed != self.previous_angular_speed:
            linear = self.max_linear_speed * self.linear_speed_reduction_factor
        else:
            linear = self.max_linear_speed

        if linear > self.max_linear_speed:
            linear = self.max_linear_speed

        self.previous_angular_speed = angular_speed
        return linear

    def calculate_angular_speed(self, cx):
        image_width = 800
        period_width = image_width / 5

        if period_width * 2 <= cx < period_width * 3:
            return 0.0  # Run straight in the middle period

        if 0 <= cx < period_width * 2:  # Range is inclined to the left
            angle_percentage = (period_width * 2 - cx) / period_width
            angular_speed = self.max_angular_speed * angle_percentage
            return -angular_speed

        if period_width * 3 <= cx < period_width * 5:  # Range is inclined to the right
            angle_percentage = (cx - period_width * 3) / period_width
            angular_speed = self.max_angular_speed * angle_percentage
            return angular_speed

        return 0.0  # Default case: Run straight

    def publish_velocity(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
