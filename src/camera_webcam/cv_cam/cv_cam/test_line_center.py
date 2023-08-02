import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__("line_follower_node")

        # Subscribe to the '/camera/image_raw' topic
        self.subscription = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.subscription

        self.publisher = self.create_publisher(Image, "/camera/cv_image", 10)
        self.bridge = CvBridge()

    def process_image(self, image):

        # Calculate the midpoint of the image
        height, width, _ = image.shape
        mid_x = width // 2        

        # Draw a straight red line through the midpoint
        cv2.line(image, (mid_x, 0), (mid_x, height), (0, 0, 255), 3)

        # Convert BGR image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define lower and upper range of red color (you may need to tune these values)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Create a mask to extract only the red regions
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the contour with the largest area (red line)
            largest_contour = max(contours, key=cv2.contourArea)

            # Calculate the centroid of the largest contour
            M = cv2.moments(largest_contour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Draw a circle at the centroid of the red line
            cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)      

        return image

    def image_callback(self, msg):
        # Convert the ROS2 Image message to a numpy array using CvBridge
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Process the received image
        processed_image = self.process_image(image)

        # Convert the processed image back to a ROS2 Image message
        processed_image_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")

        # Publish the processed image to the '/camera/cv_image' topic
        self.publisher.publish(processed_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    cv2.destroyAllWindows() 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
