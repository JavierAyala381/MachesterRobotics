#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
import time
import numpy as np

class k_model:
    def __init__(self):
        self.time = 10
        self.speed = 0.1
        self.th = 0.1
        rospy.init_node('puzz')
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    
    
    def run_rotate(self):
        if not rospy.is_shutdown():
            t = Twist()
            t.angular.z = self.th
            t_end = time.time() + self.time
            while time.time() < t_end:
                self.pub_cmd.publish(t)
                time.sleep(0.1)
            t_end = time.time()+2
            t.angular.z = 0
            while time.time() < t_end:
                self.pub_cmd.publish(t)
                time.sleep(0.1)

    def run_move(self):
        if not rospy.is_shutdown():
            t = Twist()
            t.linear.x = self.speed
            t_end = time.time() + self.time
            while time.time() < t_end:
                self.pub_cmd.publish(t)
                time.sleep(0.1)
            t_end = time.time()+2
            t.linear.x = 0
            while time.time() < t_end:
                self.pub_cmd.publish(t)
                time.sleep(0.1)


if __name__ == "__main__":
    model = k_model()
    time.sleep(3)
    model.run_rotate()