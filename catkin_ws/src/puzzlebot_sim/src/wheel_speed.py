#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import  Twist
from std_msgs.msg import Float32
from puzzlebot_info import *

class wheel_model:
    def __init__(self):
        self.L = L
        self.R = R
        self.v = 0.0
        self.w = 0.0

        rospy.init_node('puzzlebot_wheel')
        
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb)
        self.wl_pub = rospy.Publisher('/wl', Float32, queue_size=10)
        self.wr_pub = rospy.Publisher('/wr', Float32, queue_size=10)
    
    
    def cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
    
    def run(self):
        try:
            dt = 0.1
            rate = rospy.Rate(1/dt)
            while not rospy.is_shutdown():
                wr = (self.v - self.w * self.L / 2.0) / self.R
                wl = (self.v +  self.w * self.L / 2.0) /self. R
                self.wl_pub.publish(wl)
                self.wr_pub.publish(wr)
                rate.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    model = wheel_model()
    model.run()