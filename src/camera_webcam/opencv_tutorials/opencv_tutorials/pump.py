# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import math
# import numpy as np

# PUMP_DIAMETER = 360
# PISTON_DIAMETER = 90
# PISTON_COUNT = 7

# class DetectPumpNode(Node):
#     def __init__(self):
#         super().__init__('detect_pump')
#         self.get_logger().info('detect_pump node started')

#         self.bridge = CvBridge()
#         self.subscription = self.create_subscription(
#             Image,
#             'image',
#             self.process_image,
#             10
#         )
#         self.subscription  # prevent unused variable warning

#     def show_image(self, img):
#         cv2.imshow('image', img)
#         cv2.waitKey(1)

#     def plot_circles(self, img, circles, color):
#         if circles is None:
#             return

#         for (x, y, r) in circles[0]:
#             cv2.circle(img, (int(x), int(y)), int(r), color, 2)

#     def pt_dist(self, p1, p2):
#         dx = p2[0] - p1[0]
#         dy = p2[1] - p1[1]
#         return math.sqrt(dx * dx + dy * dy)

#     def pt_mean(self, p1, p2):
#         return ((int(p1[0] + p2[0]) // 2, int(p1[1] + p2[1]) // 2))

#     def rect2centerline(self, rect):
#         p0 = rect[0]
#         p1 = rect[1]
#         p2 = rect[2]
#         p3 = rect[3]
#         width = self.pt_dist(p0, p1)
#         height = self.pt_dist(p1, p2)

#         if height > width:
#             cl = (self.pt_mean(p0, p1), self.pt_mean(p2, p3))
#         else:
#             cl = (self.pt_mean(p1, p2), self.pt_mean(p3, p0))

#         return cl

#     def pt_line_dist(self, pt, line):
#         x0 = pt[0]
#         x1 = line[0][0]
#         x2 = line[1][0]
#         y0 = pt[1]
#         y1 = line[0][1]
#         y2 = line[1][1]
#         return abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / (math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)))

#     def find_angle(self, p1, p2, p3):
#         p1 = np.array(p1)
#         p2 = np.array(p2)
#         p3 = np.array(p3)
#         v1 = p1 - p2
#         v2 = p3 - p2
#         return math.atan2(-v1[0] * v2[1] + v1[1] * v2[0], v1[0] * v2[0] + v1[1] * v2[1]) * 180 / 3.14159

#     def process_image(self, msg):
#         try:
#             orig = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#             draw_img = orig

#             resized = cv2.resize(orig, None, fx=0.5, fy=0.5)
#             draw_img = resized

#             gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
#             draw_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

#             thresh_val = 150
#             ret, thresh = cv2.threshold(gray, thresh_val, 255, cv2.THRESH_BINARY)
#             draw_img = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

#             pump_radius_range = (PUMP_DIAMETER // 2 - 2, PUMP_DIAMETER // 2 + 2)
#             pump_circles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, 1, PUMP_DIAMETER, param2=7, minRadius=pump_radius_range[0], maxRadius=pump_radius_range[1])
#             self.plot_circles(draw_img, pump_circles, (255, 0, 0))
#             if pump_circles is None or len(pump_circles[0]) != 1:
#                 raise Exception("No or wrong number of pump circles found!")

#             pump_circle = pump_circles[0][0]

#             piston_area = 3.14159 * PISTON_DIAMETER ** 2 / 4
#             blob_params = cv2.SimpleBlobDetector_Params()
#             blob_params.filterByArea = True
#             blob_params.minArea = 0.80 * piston_area
#             blob_params.maxArea = 1.20 * piston_area
#             blob_detector = cv2.SimpleBlobDetector_create(blob_params)
#             blobs = blob_detector.detect(thresh)
#             draw_img = cv2.drawKeypoints(draw_img, blobs, (), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#             if len(blobs) != PISTON_COUNT:
#                 raise Exception("Wrong # of pistons: found {} expected {}".format(len(blobs), PISTON_COUNT))
#             piston_centers = [(int(b.pt[0]), int(b.pt[1])) for b in blobs]

#             im2, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#             maxC = max(contours, key=lambda c: cv2.contourArea(c))
#             boundRect = cv2.minAreaRect(maxC)
#             centerline = self.rect2centerline(cv2.boxPoints(boundRect))
#             cv2.line(draw_img, centerline[0], centerline[1], (0, 0, 255))

#             closest_piston = min(piston_centers, key=lambda ctr: self.pt_line_dist(ctr, centerline))
#             cv2.circle(draw_img, closest_piston, 5, (255, 255, 0), -1)

#             p1 = (orig.shape[1], pump_circle[1])
#             p2 = (pump_circle[0], pump_circle[1])
#             p3 = (closest_piston[0], closest_piston[1])
#             angle = self.find_angle(p1, p2, p3)
#             self.get_logger().info("Found pump angle: {}".format(angle))

#         except Exception as err:
#             self.get_logger().error(str(err))

#         self.show_image(draw_img)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DetectPumpNode()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import numpy as np

PUMP_DIAMETER = 360
PISTON_DIAMETER = 90
PISTON_COUNT = 7

class DetectPumpNode(Node):
    def __init__(self):
        super().__init__('detect_pump')
        self.get_logger().info('detect_pump node started')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.process_image,
            10
        )
        self.subscription  # prevent unused variable warning

    def show_image(self, img):
        cv2.imshow('image', img)
        cv2.waitKey(1)

    def process_image(self, msg):
        try:
            orig = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            draw_img = orig

            resized = cv2.resize(orig, None, fx=0.5, fy=0.5)
            draw_img = resized

            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            draw_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            # threshold grayscale to binary (black & white) image
            threshVal = 150
            ret,thresh = cv2.threshold(gray, threshVal, 255, cv2.THRESH_BINARY)
            drawImg = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

        except Exception as err:
            self.get_logger().error(str(err))

        self.show_image(drawImg)

def main(args=None):
    rclpy.init(args=args)
    node = DetectPumpNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
