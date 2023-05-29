#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import numpy as np
from puzzlebot_info import *


class tf_model:
    def __init__(self, prefix = ["rv/","gaz/"]):
        self.x = 0.0
        self.y = 0.0
        self.quat = Quaternion(*quaternion_from_euler(0,0,0))
        self.R = R
        self.prefix = prefix
        self.position = Pose()
        self.position.orientation = self.quat
        self.velocity = Twist()
        
        self.pub_odom_g = rospy.Publisher('/odom_gazebo', Odometry, queue_size=10)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.sub_pose_g = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_cb)
        self.tf_broadcaster = tf.TransformBroadcaster()

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x 
        self.y = msg.pose.pose.position.y 
        self.quat = msg.pose.pose.orientation

    def gazebo_cb(self,msg):
        try:
            puzzlebot_index = msg.name.index("puzzlebot")
            self.position = msg.pose[puzzlebot_index]
            self.velocity = msg.twist[puzzlebot_index]
        except:
            pass
    
    def run(self):
        try:
            dt = 0.1
            rate = rospy.Rate(1/dt)
            while not rospy.is_shutdown():
                #Rviz
                pose_trans = TransformStamped()
                pose_trans.header.stamp = rospy.Time.now()
                pose_trans.header.frame_id = "map"
                pose_trans.child_frame_id = self.prefix[0] + "base_link"

                pose_trans.transform.translation.x = self.x
                pose_trans.transform.translation.y = self.y
                pose_trans.transform.translation.z = self.R
                pose_trans.transform.rotation = self.quat
                self.tf_broadcaster.sendTransformMessage(pose_trans)

                #Gazebo
                gaz_trans = TransformStamped()
                gaz_trans.header.stamp = rospy.Time.now()
                gaz_trans.header.frame_id = "map"
                gaz_trans.child_frame_id = self.prefix[1] + "base_link"

                gaz_trans.transform.translation.x = self.position.position.x
                gaz_trans.transform.translation.y = self.position.position.y
                gaz_trans.transform.translation.z = self.position.position.z
                gaz_trans.transform.rotation = self.position.orientation
                self.tf_broadcaster.sendTransformMessage(gaz_trans)

                gaz_odom = Odometry()
                gaz_odom.header.stamp = rospy.Time.now()
                gaz_odom.header.frame_id = "map"
                gaz_odom.child_frame_id = "base_link"
                gaz_odom.pose.pose.position.x = self.position.position.x
                gaz_odom.pose.pose.position.y = self.position.position.y
                gaz_odom.pose.pose.position.z = self.position.position.z
                gaz_odom.pose.pose.orientation = self.position.orientation
                gaz_odom.twist.twist.linear = self.velocity.linear
                gaz_odom.twist.twist.angular = self.velocity.angular
                self.pub_odom_g.publish(gaz_odom)
                rate.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    rospy.init_node('puzzlebot_tf')
    model = tf_model()
    model.run()