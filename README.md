# MachesterRobotics
This repository contains the code to create a camera based Extended Kalman Filter using ArUco markers as landamars and odometry. Obstacle aboidance is also implemented using Lidar sensor. To execute follow this steps:

1. Run the following command to run Gazebo and Rviz with the world

`roslaunch puzzlebot_sim run_kalmanFilter.launch`

2. In another windows run this command to execute KalmanFilter

'''python3 EKalmanFilter.py'''

3. In another window run this command to execute navegation algorithm

'''rosrun puzzlebot_navigation_bugs bug2.py'''
