# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from geometry_msgs.msg import Twist
# import sys
# import cv2
# import numpy as np
# import time

# class Processing(Node):
#     def __init__(self):
#         super().__init__("Processing")
#         self.nodename = "Processing".upper()
#         self.get_logger().info(f"========={self.nodename}=========")


# def main(args=None):
#     rclpy.init(args=args)
#     node = Processing()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3
# import rclpy
# import cv2
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image

# def main(args=None):
#     rclpy.init(args=args)
#     node = rclpy.create_node('image_pub')
#     node.get_logger().info('image_pub node started')

#     filename = "/home/natthawe/Documents/red_line.png" 

#     img = cv2.imread(filename)
#     cv2.imshow("image", img)
#     cv2.waitKey(2000)

#     bridge = CvBridge()
#     img_msg = bridge.cv2_to_imgmsg(img, "bgr8")

#     pub = node.create_publisher(Image, 'image', 10)
#     rate = node.create_rate(1)  # 1 Hz

#     while rclpy.ok():
#         pub.publish(img_msg)
#         rate.sleep()

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

