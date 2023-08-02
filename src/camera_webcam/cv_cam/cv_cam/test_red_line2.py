import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__("line_follower_node")

        self.subscription = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.subscription

        self.publisher = self.create_publisher(Image, "/camera/cv_image", 10)
        self.bridge = CvBridge()

    def process_image(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Create a mask to extract only the red regions
        mask = cv2.inRange(hsv, lower_red, upper_red)

        print(f'mask: {mask}')
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(mask, contours, -1, (0,255,0), 3)


        # # Draw the red line on the original image
        # for contour in contours:
        #     if cv2.contourArea(contour) > 100:  # Minimum area to consider as a line (you may need to adjust this value)
        #         x, y, w, h = cv2.boundingRect(contour)
        #         cv2.line(image, (x + (w // 2), y), (x + (w // 2), y + h), (0, 0, 255), 3)    

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
