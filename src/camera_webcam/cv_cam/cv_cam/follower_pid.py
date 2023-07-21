#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

class LineDetectionNode(Node):
    def __init__(self):
        super().__init__('line_detection_node')
        self.RLine = 0
        self.point_count = 0
        self.last_point_count = 0
        self.count = 0
        self.middle = 0
        self.mark = 0
        
        self.image_pub = self.create_publisher(Image, "camera/cv_image", 10)
        self.camera_sub = self.create_subscription(
            Image, "camera/image_raw", self.process_image, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.frame_count = 0
        self.start_time = time.time()
        self.pid_controller = PIDController(kp=5.0, ki=0.0, kd=0.0, setpoint=80)
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.1

        self.linear_speed_reduction_factor = 1.0  # Adjust the reduction factor as needed
        self.previous_angular_speed = 0.0

    def process_image(self, msg):
        # Convert the ROS2 Image message to OpenCV format
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(gray_frame, (kernel_size, kernel_size), 0)
        ret, thresh = cv2.threshold(gray_frame, 80, 255, cv2.THRESH_BINARY)

        diff = []
        points = []
        start_height = []
        self.middle = 0

        # Calculate the number of rows based on image height
        num_rows = min(5, frame.shape[0] // 15)
        #num_rows = min(1, 60)
        

        for i in range(num_rows):
            start_height.append(thresh.shape[0] - 1 - (15 * i))
            signed_thresh = thresh[start_height[i]].astype(np.int16)
            diff.append(np.diff(signed_thresh))

            points.append(np.where(np.logical_or(diff[i] > 200, diff[i] < -200)))
            cv2.line(frame, (0, start_height[i]), (thresh.shape[1], start_height[i]), (255, 0, 0), 2)

            self.point_count = 0
            for j in range(len(points[i][0])):
                self.middle = points[i][0][j]
                cv2.circle(frame, (self.middle, start_height[i]), 5, (0, 0, 255), 2)
                cv2.circle(frame, (thresh.shape[1] // 2, start_height[i]), 5, (255, 0, 0), 2)
                cv2.line(frame, (self.middle, start_height[i]), (thresh.shape[1] // 2, start_height[i]), (0, 255, 0), 2)
                
                
                if i == 4:
                    self.point_count += 1
                
                # Calculate the angular speed using the PID controller
                #angular_speed = self.calculate_angular_speed(self.middle)
                # print("Output:", middle)
                # print("Angular Speed:", angular_speed)
                # Calculate the linear speed based on the angular speed
                #linear_speed = self.calculate_linear_speed(angular_speed)
                # print("Linear Speed:", linear_speed)

                # Publish the velocities
                #self.publish_velocity(linear_speed, angular_speed)
                
        if self.point_count != self.last_point_count:
            self.last_point_count = self.point_count
            print("point_count:", self.point_count)
            #print(points)

            if self.point_count >= 10:
                if self.mark == 0:
                    self.count += 1
                    self.mark = 1
                    print("Count:", self.count)
                    print("RLine:", self.RLine)
            
            if self.point_count == 2:
                self.mark = 0
                if self.count >= 2:
                    if self.RLine == 1:
                        self.RLine = 0
                    else:
                        self.RLine = 1
                    self.count = 0
                    print("Count:", self.count)
                    print("RLine:", self.RLine)
        #try:
        #print("Data = ",len(points))
        if len(points) >= 5:
            if self.RLine == 1:
                if len(points[4][0]) > 0:
                    rr = len(points[4][0])
                    self.middle = points[4][0][rr-1] 
                    #print("R5",self.middle)
            else :
                if len(points[4][0]) > 0:
                    self.middle = points[4][0][0]
                    #print("L5",self.middle)
                    
            if self.middle == 0:
                if self.RLine == 1:
                    if len(points[3][0]) > 0:
                        rr = len(points[3][0])
                        self.middle = points[3][0][rr-1] 
                        #print("R5",self.middle)
                else :
                    if len(points[3][0]) > 0:
                        self.middle = points[3][0][0]
                        #print("L5",self.middle)
            if self.middle == 0:
                if self.RLine == 1:
                    if len(points[2][0]) > 0:
                        rr = len(points[2][0])
                        self.middle = points[2][0][rr-1] 
                        #print("R5",self.middle)
                else :
                    if len(points[2][0]) > 0:
                        self.middle = points[2][0][0]
                        #print("L5",self.middle)
            if self.middle == 0:
                if self.RLine == 1:
                    if len(points[1][0]) > 0:
                        rr = len(points[1][0])
                        self.middle = points[1][0][rr-1] 
                        #print("R5",self.middle)
                else :
                    if len(points[1][0]) > 0:
                        self.middle = points[1][0][0]
                        #print("L5",self.middle)
        # [
        #     (array([134, 140]),), 
        #     (array([136, 142]),), 
        #     (array([138, 145]),), 
        #     (array([142, 149]),), 
        #     (array([147, 154]),)
        # ]
    
        #     if self.middle == 0 :
        #         if self.RLine == 1:
        #             if len(points[0][3]) > 0:
        #                 rr = len(points[0][0])
        #                 self.middle = points[0][3][rr-1] 
        #                 print("R4",self.middle)
        #         else :
        #             if len(points[0][3]) > 0:
        #                 self.middle = points[0][3][0]  
        #                 print("L4",self.middle)    
        #     if self.middle == 0 :
        #         if self.RLine == 1:
        #             if len(points[0][2]) > 0:
        #                 rr = len(points[0][0])
        #                 self.middle = points[0][2][rr-1] 
        #                 print("R3",self.middle)
        #         else :
        #             if len(points[0][2]) > 0:
        #                 self.middle = points[0][2][0]  
        #                 print("L3",self.middle) 
        #     if self.middle == 0 :
        #         if self.RLine == 1:
        #             if len(points[0][1]) > 0:
        #                 rr = len(points[0][0])
        #                 self.middle = points[0][1][rr-1] 
        #                 print("R2",self.middle) 
        #         else :
        #             if len(points[0][1]) > 0:
        #                 self.middle = points[0][1][0]    
        #                 print("L2",self.middle) 
        # #except:
        #    pass 
        # if self.RLine == 1:
        #     if len(points) > 0 and len(points[0]) > 1:
        #         self.middle = (points[0][0] + points[0][1]) // 2
        #         print("GoR")
        # else:
        #     if len(points) > 1 and len(points[1]) > 1:
        #         self.middle = (points[1][0] + points[1][1]) // 2
        #         print("GoL")
        #     elif len(points) > 0 and len(points[0]) > 1:
        #         self.middle = (points[0][0] + points[0][1]) // 2
                


        if self.middle == 0:
            self.get_logger().info("Not found!!!")
            self.publish_velocity(0.0, 0.0)
        else:   
            angular_speed = self.calculate_angular_speed(self.middle)
            linear_speed = self.calculate_linear_speed(angular_speed)
            self.publish_velocity(linear_speed, angular_speed)
        
        img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(img_msg)

    def calculate_linear_speed(self, angular_speed):
        if self.previous_angular_speed != 0.0 and angular_speed != self.previous_angular_speed:
            linear = self.max_linear_speed * self.linear_speed_reduction_factor
        else:
            linear = self.max_linear_speed

        if linear > self.max_linear_speed:
            linear = self.max_linear_speed

        self.previous_angular_speed = angular_speed
        return linear

    def calculate_angular_speed(self, pp):
        err = (pp - 80)*(-1)
        val = err / 800
        return val

    def publish_velocity(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    rclpy.spin(node)
    cv2.destroyAllWindows() 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
