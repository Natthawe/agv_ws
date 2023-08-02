import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

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
          
# --------------
            # Draw contours on the original image
            cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)

            # Thin image to find clear contours
            thin = cv2.ximgproc.thinning(red_mask, thinningType=cv2.ximgproc.THINNING_GUOHALL)

            # dind contours
            cnts = cv2.findContours(thin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            # print(f'cnts: {cnts}')
            cnts = cnts[0] if len(cnts) == 2 else cnts[1]
            c = max(cnts, key=cv2.contourArea)
            cv2.drawContours(cv_image, [c], -1, (255, 255, 0), 2)            

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
