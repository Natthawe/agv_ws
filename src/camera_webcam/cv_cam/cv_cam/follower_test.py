import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineDetectionNode(Node):
    def __init__(self):
        super().__init__('line_detection_node')
        self.bridge = CvBridge()
        self.LeftLine = True
        self.last_point_count = 0
        self.count = 0

        # Create a subscriber to receive the video stream
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',  # Replace 'camera/image' with your video topic
            self.process_image,
            10
        )
        self.subscription

    def process_image(self, msg):
        # Convert the ROS2 Image message to OpenCV format
        frame_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame = frame_rgb.copy()

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(frame, (kernel_size, kernel_size), 0)
        ret, thresh = cv2.threshold(blur_gray, 80, 255, cv2.THRESH_BINARY)

        diff = []
        points = []
        start_height = []

        # Calculate the number of rows based on image height
        num_rows = min(5, frame_rgb.shape[0] // 45)

        for i in range(num_rows):
            start_height.append(thresh.shape[0] - 1 - (45 * i))
            signed_thresh = thresh[start_height[i]].astype(np.int16)
            diff.append(np.diff(signed_thresh))

            points.append(np.where(np.logical_or(diff[i] > 200, diff[i] < -200)))
            cv2.line(frame_rgb, (0, start_height[i]), (thresh.shape[1], start_height[i]), (255, 0, 0), 2)

            point_count = 0
            for j in range(len(points[i][0])):
                middle = points[i][0][j]
                cv2.circle(frame_rgb, (middle, start_height[i]), 5, (0, 0, 255), 2)
                cv2.circle(frame_rgb, (thresh.shape[1] // 2, start_height[i]), 5, (255, 0, 0), 2)
                cv2.line(frame_rgb, (middle, start_height[i]), (thresh.shape[1] // 2, start_height[i]), (0, 255, 0), 2)
                point_count += 1

        if point_count != self.last_point_count:
            self.last_point_count = point_count
            print(point_count)

            if point_count == 4:
                self.count += 1
            
            if point_count == 2:
                if self.count >= 2:
                    self.LeftLine = not self.LeftLine
                    self.count = 0

        if self.LeftLine:
            cv2.putText(frame_rgb, 'Left:', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        else:
            cv2.putText(frame_rgb, 'Right:', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # Display count value
        cv2.putText(frame_rgb, f'Count: {self.count}', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # Display the image with lines and count value
        cv2.imshow('Video', frame_rgb)
        cv2.imshow('Videoth', thresh)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    rclpy.spin(node)
    cv2.destroyAllWindows() 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
