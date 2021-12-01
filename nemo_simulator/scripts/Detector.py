#!/usr/bin/env python3

if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    
import sys
import rospy
import cv2
import time

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from include.img_cleaning   import *


class Detector:
    def __init__(self, hsv_min, hsv_max):

        self.set_hsv = (hsv_min, hsv_max)
        self.nemo_point = Point()

        self.image_pub = rospy.Publisher("/marlin/image_nemo",Image,queue_size=3)
        self.mask_pub = rospy.Publisher("/marlin/image_nemo_mask",Image,queue_size=3)

        self.marlin_pub  = rospy.Publisher("/marlin/nemo_point",Point,queue_size=3)

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print("Frame Dropped: ", e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            keypoints, mask   = cleaning(cv_image, self.set_hsv[0], self.set_hsv[1])

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))
            except CvBridgeError as e:
                print(e)
            
            for i, keyPoint in enumerate(keypoints):

                x = keyPoint.pt[0]
                y = keyPoint.pt[1]
                s = keyPoint.size
                #print ("kp %d: s = %3d   x = %3d  y= %3d"%(i, s, x, y))
                
                x, y = get_nemo_relative_position(cv_image, keyPoint)
                
                self.nemo_point.x = x
                self.nemo_point.y = y
                self.marlin_pub.publish(self.nemo_point) 
                break

def main(args):
    hsv_min = (31, 150, 100)
    hsv_max = (71, 255, 255)

    ic = Detector(hsv_min, hsv_max)
    rospy.init_node('nemo_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)