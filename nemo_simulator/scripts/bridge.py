import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraHandler():

    def __init__(self):
        self.sub = rospy.Subscriber("camera/image_raw", Image, self.callback)
        self.pub=rospy.Publisher("my_image",Image,queue_size=10)

    def callback(self,msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        self.pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    
def main():
    rospy.init_node('my_image')
    CameraHandler()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

