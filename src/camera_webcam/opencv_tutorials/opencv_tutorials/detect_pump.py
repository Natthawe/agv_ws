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

class PumpDetectionNode(Node):
    def __init__(self):
        super().__init__('detect_pump')
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_subscription = self.create_subscription(
            Image,
            'image',
            self.process_image,
            10
        )

    def process_image(self, msg):
        try:
            # convert sensor_msgs/Image to OpenCV Image
            bridge = CvBridge()
            orig = bridge.imgmsg_to_cv2(msg, "bgr8")
            drawImg = orig

            # resize image (half-size) for easier processing
            resized = cv2.resize(orig, None, fx=0.5, fy=0.5)
            drawImg = resized

            # convert to single-channel image
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            drawImg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            # threshold grayscale to binary (black & white) image
            threshVal = 150
            ret,thresh = cv2.threshold(gray, threshVal, 255, cv2.THRESH_BINARY)
            drawImg = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

            # detect outer pump circle
            pumpRadiusRange = (int(PUMP_DIAMETER/2-2), int(PUMP_DIAMETER/2+2))
            pumpCircles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, 1, PUMP_DIAMETER, param1=5, param2=7, minRadius=pumpRadiusRange[0], maxRadius=pumpRadiusRange[1])
            self.plotCircles(drawImg, pumpCircles, (255,0,0))

            if (pumpCircles is None):
                raise Exception("No pump circles found!")
            elif len(pumpCircles[0]) != 1:
                raise Exception("Wrong # of pump circles: found {} expected {}".format(len(pumpCircles[0]),1))
            else:
                pumpCircle = pumpCircles[0][0]

            # detect blobs inside pump body
            pistonArea = 3.14159 * PISTON_DIAMETER**2 / 4
            blobParams = cv2.SimpleBlobDetector_Params()
            blobParams.filterByArea = True
            blobParams.minArea = 0.80 * pistonArea
            blobParams.maxArea = 1.20 * pistonArea
            blobDetector = cv2.SimpleBlobDetector_create(blobParams)
            blobs = blobDetector.detect(thresh)
            drawImg = cv2.drawKeypoints(drawImg, blobs, (), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            if len(blobs) != PISTON_COUNT:
                raise Exception("Wring # of pistons: found {} expected {}".format(len(blobs), PISTON_COUNT))
            pistonCenters = [(int(b.pt[0]),int(b.pt[1])) for b in blobs]

            # determine primary axis, using largest contour
            im2, contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            maxC = max(contours, key=lambda c: cv2.contourArea(c))
            boundRect = cv2.minAreaRect(maxC)
            centerline = rect2centerline(cv2.boxPoints(boundRect))
            cv2.line(drawImg, centerline[0], centerline[1], (0,0,255))

            # find closest piston to primary axis
            closestPiston = min( pistonCenters, key=lambda ctr: ptLineDist(ctr, centerline))
            cv2.circle(drawImg, closestPiston, 5, (255,255,0), -1)

            # calculate pump angle
            p1 = (orig.shape[1], pumpCircle[1])
            p2 = (pumpCircle[0], pumpCircle[1])
            p3 = (closestPiston[0], closestPiston[1])
            angle = findAngle(p1, p2, p3)
            print("Found pump angle: {}".format(angle))

        except Exception as err:
            self.get_logger().error(str(err))

        # Show results
        self.show_image(drawImg)

    def show_image(self, drawImg):
        cv2.imshow('image', drawImg)
        cv2.waitKey(1)


    def plotCircles(self, drawImg, circles, color):
        if circles is None: return

        for (x,y,r) in circles[0]:
            cv2.circle(drawImg, (int(x),int(y)), int(r), color, 2)

    def ptDist(self, p1, p2):
        dx=p2[0]-p1[0]; dy=p2[1]-p1[1]
        return math.sqrt( dx*dx + dy*dy )

    def rect2centerline(self, rect):
        p0=rect[0]; p1=rect[1]; p2=rect[2]; p3=rect[3];
        width=ptDist(p0,p1); height=ptDist(p1,p2);

        # centerline lies along longest median
        if (height > width):
            cl = ( ptMean(p0,p1), ptMean(p2,p3) )
        else:
            cl = ( ptMean(p1,p2), ptMean(p3,p0) )

        return cl

def main(args=None):
    rclpy.init(args=args)
    pump_detection_node = PumpDetectionNode()
    rclpy.spin(pump_detection_node)
    pump_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
