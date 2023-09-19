#!/usr/bin/env python3
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import argparse

def rotateImg(img, angle):
    rows, cols, ch = img.shape
    M = cv2.getRotationMatrix2D((cols / 2, rows / 2), angle, 1)
    return cv2.warpAffine(img, M, (cols, rows))

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='Publish rotated images')
    parser.add_argument('filename', help='Path to the image file')
    args = parser.parse_args()

    node = rclpy.create_node('image_pub')
    node.get_logger().info('image_pub node started')

    filename = args.filename
    img = cv2.imread(filename)

    bridge = CvBridge()
    imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")

    pub = node.create_publisher(Image, 'image', 10)

    angle = 0
    while rclpy.ok():
        rotImg = rotateImg(img, angle)
        imgMsg = bridge.cv2_to_imgmsg(rotImg, "bgr8")

        pub.publish(imgMsg)
        angle = (angle + 10) % 360
        rclpy.spin_once(node, timeout_sec=1.0)  # Spin at 1 Hz

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
