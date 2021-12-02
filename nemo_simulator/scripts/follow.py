#!/usr/bin/python3

import math, time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

K_LAT_DIST_TO_STEER     = 2.0

def saturate(value, min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)

class Follower:
    def __init__(self):
        self.nemo_x = 0.0
        self.nemo_y = 0.0
        self.time_detected = 0.0

        self.sub = rospy.Subscriber("/marlin/nemo_point", Point, self.update_pos)

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

        self.msg = Twist()

    @property
    def is_detected(self):
        return(time.time() - self.time_detected < 1.0)

    def update_pos(self, pos):
        self.nemo_x = pos.x
        self.nemo_y = pos.y
        self.time_detected = time.time()

    def controller(self):
        steer_action = 0.0
        throttle_action = 0.0
        if (self.nemo_x == 0.0):
            steer_action = 0.0
        elif (self.nemo_x > 637.0):
            steer_action = -1.5
        elif (self.nemo_x < 637.0):
            steer_action = 1.5
        else:
            steer_action = 0.0

        if (self.nemo_y == 0.0):
            throttle_action = 0.0
        elif (self.nemo_y > 227.0):
            throttle_action = 2.0
        else:
            throttle_action = 0.0
        
        return(steer_action, throttle_action)
        
    def run(self):
        
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            
            steer_action, throttle_action = self.controller() 
            print(steer_action, throttle_action)
            
            self.msg.linear.y  = throttle_action
            self.msg.angular.z = steer_action
            
            print(steer_action, throttle_action)
            self.pub_twist.publish(self.msg)

            rate.sleep()  


if __name__ == "__main__":

    rospy.init_node('Follower')
    
    chase_ball = Follower()
    chase_ball.run()  

