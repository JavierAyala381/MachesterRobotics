#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from std_msgs.msg import Float32
from math import cos, sin
import numpy as np
x = 0.0
y = 0.0
th = 0.0

v = 0.0
w = 0.0

L = 0.18
R = 0.05
wr = 0
wl = 0


def get_wr(msg):
    global wr
    wr = msg.data

def get_wl(msg):
    global wl
    wl = msg.data

if __name__ == '__main__':

    rospy.init_node('odometry_publisher')

    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
    wr_sub = rospy.Subscriber("/wr",Float32,callback=get_wr)
    wl_sub = rospy.Subscriber("/wl",Float32,callback=get_wl)
    odom_broadcaster = tf.TransformBroadcaster()


    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(10)

    mu = np.zeros((3,1))
    sigma = np.zeros((3,3))
    Q = np.zeros((3,3))
    Q[0,:] = [0.5, 0.01, 0.01]
    Q[1,:] = [0.01, 0.5, 0.01]
    Q[2,:] = [0.01, 0.01, 0.5]
    
    while not rospy.is_shutdown():
        #print(wr,wl)
        current_time = rospy.Time.now()
        w = R * (wr - wl) / L
        v = R * (wr + wl) * 0.5
        # compute odometry in a typical way given the velocities of the robot
        dt = (last_time-current_time).to_sec()
        delta_x = (v * cos(th)) * dt
        delta_y = (v * sin(th)) * dt
        delta_th = w * dt

        x += delta_x
        y += delta_y
        th += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, th + np.pi))

        prev_mu = mu.copy()
        mu[0] = mu[0] + dt * v * np.cos(mu[2])
        mu[1] = mu[1] + dt * v * np.sin(mu[2])
        mu[2] = mu[2] + dt* mu[2]

        H = [[1,0,-dt*v*np.sin(prev_mu[2])],[0, 1, dt*v*np.cos(prev_mu[2])],[0,0,1]]
        H = np.array(H)
        sigma = H * sigma *H.T + Q
        # first, we'll publish the transform over tf
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        odom_trans.transform.translation.x = x
        odom_trans.transform.translation.y = y
        odom_trans.transform.translation.z = th
        odom_trans.transform.rotation = odom_quat

        # send the transform
        odom_broadcaster.sendTransformMessage(odom_trans)

        map_trans = TransformStamped()
        map_trans.header.stamp = current_time
        map_trans.header.frame_id = "map"
        map_trans.child_frame_id = "odom"

        map_trans.transform.translation.x = 0
        map_trans.transform.translation.y = 0
        map_trans.transform.translation.z = 0
        o = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
        map_trans.transform.rotation = o

        # send the transform
        odom_broadcaster.sendTransformMessage(map_trans)
        
        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat
        
        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = w

        # publish the message
        odom_pub.publish(odom)
        last_time = current_time
        r.sleep()