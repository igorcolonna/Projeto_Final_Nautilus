#!/usr/bin/python3

import math, time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

class Follower:

    def __init__(self):
        self.nemo_x = 0.0
        self.nemo_y = 0.0
        self.time_detected = 0.0
        
        self.marlin_x = 0.0
        self.marlin_y = 0.0

        self.sub = rospy.Subscriber("/marlin/nemo_point", Point, self.update_pos)
        self.sub2 = rospy.Subscriber("sonar_data", PointStamped, self.update_sonar)

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

        self.msg = Twist()

    def update_pos(self, pos):
        self.nemo_x = pos.x
        self.nemo_y = pos.y
        self.time_detected = time.time() 
    
    def controller(self):
        steer_action = 0.0
        throttle_action = 0.0
        if (self.nemo_x == 0.0):
            steer_action = self.adjuster()
        elif (self.nemo_x > 637.0):
            steer_action = -1.5
        elif (self.nemo_x < 637.0):
            steer_action = 1.5
        else:
            steer_action = 0.0

        if (self.nemo_y == 0.0):
            throttle_action = 0.0
        elif (self.nemo_y > 227.0):
            throttle_action = 3.0
        else:
            throttle_action = 0.0
        
        return(steer_action, throttle_action)

    def update_sonar(self, msg):
        self.marlin_x = msg.point.x
        self.marlin_y = msg.point.y

    
    def adjuster(self):
        try:
            distance = math.sqrt(self.marlin_x**2 + self.marlin_y**2)
            self.sin = self.marlin_y / distance
            theta = math.asin(self.sin)
            # def of rotation angle will change
            rotation_angle = theta + math.pi / 2
            return rotation_angle
        except ZeroDivisionError:
            print("deu ruim")
            return 0.0
                
    def run(self):

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            
            steer_action, throttle_action = self.controller() 
          
            self.msg.linear.y  = throttle_action
            self.msg.angular.z = steer_action

            self.pub_twist.publish(self.msg)

            rate.sleep()  



if __name__ == "__main__":
    time.sleep(1)
    rospy.init_node('Follower')
    
    chase_nemo = Follower()
    chase_nemo.run()  

