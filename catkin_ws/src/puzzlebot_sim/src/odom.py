#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
from math import cos, sin
import numpy as np
from puzzlebot_info import *


class covariance_generator:
    def __init__(self):
        self.kr = 1.0
        self.kl = 1.8

        self.wl = 0.0
        self.wr = 0.0

        self.mu = np.zeros((3,1),dtype=float)
        self.sigma = np.zeros((3,3),dtype=float)
        self.H = np.zeros((3,3),dtype=float)

        self.sigma_delta = np.array([[self.kr * abs(self.wr), 0], [0, self.kl * abs(self.wl)]],dtype=float)
        self.Q = np.zeros((3,3),dtype=float)
    
    def get_cov(self, wl, wr, v, sk_1, dt):
        self.wl = wl
        self.wr = wr
        x = 0
        y = 1
        theta = 2
        gra_wk = 0.5 * R * dt * np.array([[np.cos(sk_1[theta]), np.cos(sk_1[theta])],[np.sin(sk_1[theta]), np.sin(sk_1[theta])],[2.0/L, -2.0/L]])
        self.sigma_delta = np.array([[self.kr * abs(self.wr), 0.0], [0.0, self.kl * abs(self.wl)]])
        self.Q = np.matmul(np.matmul(gra_wk,self.sigma_delta), gra_wk.T)
        
        self.H = np.array([[1.0,0.0, -dt * v * np.sin(sk_1[theta])],[0.0,1.0,dt * v * np.cos(sk_1[theta])],[0.0,0.0,1.0]])

        self.sigma = np.matmul(np.matmul(self.H, self.sigma),self.H.T) + self.Q
        
        return self.sigma

class k_model:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v = 0.0
        self.w = 0.0
        self.wr = 0.0
        self.wl = 0.0

        self.cov = covariance_generator()

        rospy.init_node('puzzlebot_deadReckoning')
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.wl_sub = rospy.Subscriber('/wl', Float32, self.wr_cb)
        self.wr_sub = rospy.Subscriber('/wr', Float32, self.wl_cb)

    
    
    def wr_cb(self, msg):
        self.wr = msg.data

    def wl_cb(self, msg):
        self.wl = msg.data
    
    def run(self):
        try:
            dt = 0.1
            past_t = rospy.Time.now()
            
            rate = rospy.Rate(1/dt)
            rate.sleep()
            while not rospy.is_shutdown():
                now_t = rospy.Time.now()
                dt = (now_t - past_t).to_sec()
                self.w = R * (self.wr - self.wl) / L
                self.v = R * (self.wr + self.wl) * 0.5
                
                self.th += self.w * dt
                self.x += self.v * np.cos(self.th) * dt
                self.y += self.v * np.sin(self.th) * dt
                sigma = self.cov.get_cov(self.wl,self.wr,self.v,[self.x,self.y,self.th],dt)
                o = Odometry()
                o.header.frame_id = "map"
                o.child_frame_id = "base_link"
                o.header.stamp = now_t
                o.pose.pose.position.x = self.x
                o.pose.pose.position.y = self.y

                quat = Quaternion(*quaternion_from_euler(0,0,self.th))
                o.pose.pose.orientation = quat
                
                co = np.zeros((6,6),dtype=float)
                co[:2,:2] = sigma[:2,:2]
                co[-1,:2] = sigma[-1,:2]
                co[:2,-1] = sigma[:2,-1]
                co[-1,-1] = sigma[-1,-1]
                o.pose.covariance = co.reshape(36).tolist()

                o.twist.twist.linear.x = self.v
                o.twist.twist.angular.z = self.w
                self.pub_odom.publish(o)
                past_t = now_t
                rate.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    model = k_model()
    model.run()



