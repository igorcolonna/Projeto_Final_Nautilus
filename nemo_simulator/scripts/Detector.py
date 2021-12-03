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

        position = self.position(self.filtro(cv_image))
        self.image_pub.publish(bridge.cv2_to_imgmsg(position[0], "8UC1"))
        self.nemo_point.x = position[1][0]
        self.nemo_point.y = position[1][1]
        self.marlin_pub.publish(self.nemo_point)
        

    def filtro(self, img):

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,self.set_hsv[0], self.set_hsv[1])

        return mask

    def position(self,mask):
        
        contours,hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        blank = np.zeros(mask.shape[:2], dtype='uint8')
        cx = 0
        cy = 0
        cv2.drawContours(blank, contours, -1, (255, 0, 0), 1)
        reversemask = 255-mask
        for i in contours:
            M = cv2.moments(i)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.drawContours(reversemask, [i], -1, (0, 255, 0), 2)
                cv2.circle(reversemask, (cx, cy), 7, (255, 255, 255), -1)
                cv2.putText(reversemask,"center", (cx - 20, cy - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            print(f"x: {cx} y: {cy}")
        
        coord = [cx, cy]

        return reversemask, coord


    

if __name__ == '__main__':
    hsv_min = (31, 150, 100)
    hsv_max = (71, 255, 255)

    rospy.init_node('nemo_detector', anonymous=True)

    ic = Detector(hsv_min, hsv_max)
    
    rospy.spin()