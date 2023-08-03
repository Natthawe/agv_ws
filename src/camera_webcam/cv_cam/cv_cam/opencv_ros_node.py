# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# class OpenCVROSNode(Node):
#     def __init__(self):
#         super().__init__('opencv_ros_node')
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/image_raw',  # Subscribe to the raw camera image topic
#             self.image_callback,
#             10
#         )
#         self.publisher = self.create_publisher(
#             Image,
#             '/camera/cv_image',  # Publish the processed image to this topic
#             10
#         )
#         self.bridge = CvBridge()

#     def image_callback(self, msg):
#         try:
#             # Convert ROS2 image message to OpenCV image
#             cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

#             hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

#             # Define the range for red color in HSV format
#             lower_red1 = np.array([0, 100, 100])
#             upper_red1 = np.array([10, 255, 255])
#             lower_red2 = np.array([160, 100, 100])
#             upper_red2 = np.array([180, 255, 255])

#             # Create masks for the two ranges of red color (since red hue wraps around 0-degree mark)
#             red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
#             red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

#             # Combine the masks to get the final red mask
#             red_mask = red_mask1 + red_mask2            

#             # Find contours in the red mask
#             contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
          
# # --------------
#             # Draw contours on the original image
#             cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)

#             # Thin image to find clear contours
#             thin = cv2.ximgproc.thinning(red_mask, thinningType=cv2.ximgproc.THINNING_GUOHALL)

#             # dind contours
#             cnts = cv2.findContours(thin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#             # print(f'cnts: {cnts}')
#             cnts = cnts[0] if len(cnts) == 2 else cnts[1]
#             c = max(cnts, key=cv2.contourArea)
#             cv2.drawContours(cv_image, [c], -1, (255, 255, 0), 2)            

#             # Convert the processed OpenCV image back to a ROS2 image message
#             processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')

#             # Publish the processed image
#             self.publisher.publish(processed_image_msg)

#         except Exception as e:
#             self.get_logger().error('Error processing image: %s' % str(e))

# def main(args=None):
#     rclpy.init(args=args)
#     opencv_ros_node = OpenCVROSNode()
#     rclpy.spin(opencv_ros_node)
#     opencv_ros_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import math

class OpenCVROSNode(Node):
    def __init__(self):
        super().__init__('opencv_ros_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Subscribe to the raw camera image topic
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            Image,
            '/camera/cv_image',  # Publish the processed image to this topic
            10
        )
        
        # Create a publisher to publish the calculated angle in radians
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS2 image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

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
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # Draw contours on the original image
            cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)

            # Thin image to find clear contours
            thin = cv2.ximgproc.thinning(red_mask, thinningType=cv2.ximgproc.THINNING_GUOHALL)

            # Find contours again
            cnts = cv2.findContours(thin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnts = cnts[0] if len(cnts) == 2 else cnts[1]

            if cnts:  # Check if contours were found
                c = max(cnts, key=cv2.contourArea)

                # Fit a line to the contour points
                rows, cols = thin.shape
                [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
                lefty = int((-x * vy / vx) + y)
                righty = int(((cols - x) * vy / vx) + y)

                # # Calculate the angle of the line with respect to the horizontal axis
                # angle_rad = math.atan2(lefty - righty, cols - 1)
                # angle_deg = math.degrees(angle_rad)

                # # Adjust the angle to have 0 degrees at the center and range from +90 to -90 degrees
                # angle_deg -= 90  # Shift the range by -90 degrees
                # if angle_deg < -90:  # Normalize angle to the range from +90 to -90 degrees
                #     angle_deg += 180

                # print(f"Line angle (degrees): {angle_deg}")

                # Calculate the angle of the line with respect to the horizontal axis
                angle_rad = math.atan2(lefty - righty, cols - 1)
                angle_rad -= math.pi / 2  # Shift the range by -pi/2 radians
                if angle_rad < -math.pi / 2:  # Normalize angle to the range from +pi/2 to -pi/2 radians
                    angle_rad += math.pi

                # print(f"Line angle (radians): {angle_rad}")

                # Create a Twist message to publish the calculated angle
                twist_msg = Twist()
                twist_msg.linear.x = 0.3
                twist_msg.angular.z = angle_rad

                # Publish the Twist message on the cmd_vel topic
                self.cmd_vel_publisher.publish(twist_msg)

                # Add the line as a contour
                line_contour = np.array([[[cols - 1, righty]], [[0, lefty]]])

                # Draw the line using cv2.drawContours()
                cv2.drawContours(cv_image, [line_contour], -1, (255, 255, 0), 2)          

            # Convert the processed OpenCV image back to a ROS2 image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')

            # Publish the processed image
            self.publisher.publish(processed_image_msg)

        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    opencv_ros_node = OpenCVROSNode()
    rclpy.spin(opencv_ros_node)
    opencv_ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
