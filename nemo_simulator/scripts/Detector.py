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
from cv_bridge              import CvBridge
import numpy as np
#from img_cleaning           import *


class Detector:
    def __init__(self, hsv_min, hsv_max):

        self.set_hsv = (hsv_min, hsv_max)
        self.nemo_point = Point()

        self.image_pub = rospy.Publisher("/marlin/image_nemo",Image,queue_size=1)
        self.mask_pub = rospy.Publisher("/marlin/image_nemo_mask",Image,queue_size=1)

        self.marlin_pub  = rospy.Publisher("/marlin/nemo_point",Point,queue_size=10)

        self.sub = rospy.Subscriber("camera/image_raw", Image, self.callback)
        self.pub=rospy.Publisher("my_image",Image,queue_size=10)

    def callback(self, msg):

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        self.pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        self.mask_pub.publish(bridge.cv2_to_imgmsg(self.filtro(cv_image), "8UC1"))
        #self.position(self.filtro(cv_image), cv_image)
        #self.image_pub.publish(bridge.cv2_to_imgmsg(self.position(self.filtro(cv_image),cv_image), "8UC1"))


    def filtro(self, img):

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,self.set_hsv[0], self.set_hsv[1])

        return mask
    

if __name__ == '__main__':
    hsv_min = (31, 150, 100)
    hsv_max = (71, 255, 255)

    rospy.init_node('nemo_detector', anonymous=True)

    ic = Detector(hsv_min, hsv_max)
    
    rospy.spin()
    #try:
    
    #except KeyboardInterrupt:
    #   print("Shutting down")

    #main(sys.argv)