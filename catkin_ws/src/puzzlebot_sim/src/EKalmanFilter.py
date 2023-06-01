#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import cv2, PIL
import matplotlib.pyplot as plt
import matplotlib as mpl
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist, Quaternion, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv2 import aruco

class ExtendedKalmanFIlter:
    def __init__(self):
        #Create a Publisher to the EKF node
        #self.ekf_pub = rospy.Publisher('/ekf_estimate', PoseWithCovarianceStamped, queue_size=1)

        #Initialize the node
        rospy.init_node('ExtendedKalmanFilter')

        # Set Camera Parameters

        # Camera Matrix - matrix of 3x3 elements with the focal distance and the camara center cordinates 
        self.cameraMatrix = np.array([
        [476.7030836014194, 0.0, 400.5],
        [0.0, 476.7030836014194, 400.5],
        [0.0, 0.0, 1.0]], 
        dtype=np.float32
        )

        # Distortion Coefficients . a vector with 5 or more elements that model the distortion of the camara
        self.distCoeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

        # Length of the size of the marker in meters
        # In thi case 0.05 for aruco.DICT_6X6_250
        #self.markerLength = 0.05
        self.markerLength = 0.250

        #Subscribe to ROS sensor Nodes
        self.bridge = CvBridge()
        self.odom_pub = rospy.Publisher('/odom_calc', PoseWithCovarianceStamped, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.odom_sub = rospy.Subscriber('/odom_gazebo', Odometry, self.odom_callback)
        self.cmd_vel = rospy.Subscriber('/cmd_vel',Twist, self.cmd_vel_callback)

        #------------Initialize EKF variables-------------------

        # Create an empty hash table
        self.hash_table = {}

        # Set landmarks array
        self.landMarks = np.array([
            [2.0, 0.0],
            [5.0, 0.0],
            [4.5, 2.8],
            [2.3, 4.4],
            [0.0, 2.0]
        ])

        # Iterate over the elements of the array and add them to the hash table
        for i, landmark in enumerate(self.landMarks):
            id = str(i + 1)  # Generate an ID by incrementing the index by 1
            self.hash_table[id] = landmark

        # Set the Aruco measurment
        self.ArUcoLocation = np.zeros(2)
        '''
        Assume sample timining dt, angular and lineal velocity as constants
        '''

        # Set initil conditions
        self.velocity = 0.0 # In m/s
        self.angular = 0.0 # In rad/s
        self.dt = 0.1 # In s

        '''
        Initial state of the robot starts at [0,0,0]
        The state vector represents the current state of the robot and include has the form:
            [x,y,theta] 
        not that velocity is asume to be constantodom_gazebo
        '''
        self.state = np.array([0.08,0,0])
        self.estimatedState = np.zeros(3)
        self.estimatedLM = np.array(2)
        self.G = np.zeros((2,3))

        '''
        Set initial value for Kalman Gain
        '''
        self.kalmanGain = np.zeros(3)

        '''
        Initial covariance matrix is 0, there is low or no uncertanty
        since we know we initial position of the robot
        ''' 
        #self.covariance = np.zeros(3) 
        self.covariance = np.zeros((3,3))

        # Montion covariance model matrix
        self.Q = np.array([
            [0.5, 0.01, 0.01],
            [0.01, 0.5, 0.01],
            [0.01, 0.01, 0.2]
        ])

        # Observation cavariance model
        self.R = np.array([
            [0.02, 0],
            [0, 0.02]
        ])
        #--------------------------------------------------------
        self.execute = False
        self.ArUco_detected = False
        self.robotState = 'Waiting'
        self.tvec=np.array(3)
        self.closestMarker = np.array(2)

    def init_goal_set(self, goals):
        # Iterate over the array of arrays
        for array in goals:
            # Extract x and y coordinates from the array
            x = array[0]
            y = array[1]
            
            # Create a PoseStamped message
            pose_msg = PoseStamped()
            
            # Set the position of the pose
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.0  # Assuming z coordinate is zero
            
            # Set the orientation (quaternion)
            pose_msg.pose.orientation.w = 1.0  # Default orientation
            
            # Set the header of the pose
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "base_link"  # Set the frame of reference
            
            # Publish the PoseStamped message
            self.goal_pub.publish(pose_msg)

            # Sleep for a short duration between each pose publication
            rospy.sleep(0.1)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detect_ArUco(cv_image)
            #cv2.imshow("Camera",cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(e)

    def odom_callback(self, msg):
        try:
            #Extract odometry data
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            theta = msg.pose.pose.orientation.z

            odometryData = np.array([x,y,theta])

            print("Debug - : ", np.linalg.norm(self.state-odometryData), self.robotState)
            if np.linalg.norm(self.state-odometryData) < 0.009:
                self.robotState = 'Stop'
            else:
                self.robotState = 'Run'

            #Update robot State variables
            self.state[0] = odometryData[0]
            self.state[1] = odometryData[1]
            self.state[2] = odometryData[2]

        except Exception as e:
            rospy.logerr(e)

    def cmd_vel_callback(self, msg):
        self.velocity = msg.linear.x
        self.angular = msg.angular.z

    def detect_ArUco(self, frame):
        # Frame post procesing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Create Aruco Dictionary. The main properties of the dictonary are
        # the dictionary size and marker size (numer of bits)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        # Create the DetectorParameters object
        parameters =  aruco.DetectorParameters_create()
        # Responsible for detecting aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Check if any ArUco Marker was detected
        if corners is not None and ids is not None:
            # Chose a marker to start
            self.closestMarker = self.hash_table[str(ids[0][0])]
            closestMarker_ID = str(ids[0][0])
            for i in ids:
                print("Marker with id: ", i[0], "in position: ", self.hash_table[str(i[0])])
                if np.linalg.norm(self.hash_table[str(i[0])]) < np.linalg.norm(self.closestMarker):
                    self.closestMarker = self.hash_table[str(i[0])]
                    closestMarker_ID = str(i[0])
            print("Closest Marker with Id: ", closestMarker_ID, "in position: ", self.closestMarker)
            

            for i in range(0, len(ids)): #Do for each ArUco marcker
                '''
                Tvec is the vector from the camera frame to the center of the ArUco marker
                Rvec is the rotation of the marker viewed from the camera frame
                Since Tvec and Rvec are expresed in the camera frame which is described as follows
                    - z-> is defined as the optical axis of the camera +Z is in front of the camera
                    - x -> +x points to the right
                    - y -> +y points down
                z in the camera frame is x in the base frame as well y-> z and x->y
                '''
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(
                    corners[i], 
                    self.markerLength,
                    self.cameraMatrix,
                    self.distCoeffs
                )

                # Draw Auro Markers Axis
                cv2.drawFrameAxes(
                    frame, 
                    self.cameraMatrix,
                    self.distCoeffs,
                    rvec,
                    tvec, 
                    self.markerLength * 0.5
                )  

                # Draw a square and ids around the markers
                aruco.drawDetectedMarkers(frame, corners, ids) 
                
                if str(ids[i][0]) == closestMarker_ID:
                    self.tvec = tvec

                self.ArUco_detected=True
                cv2.imshow("ArUco marker detector", frame)
        else:
            self.ArUco_detected = False
            print("Warning: No ArUco markers found")

    def predict(self):
        if self.robotState == 'Run':
            ''' 
            How to predict?
            Predicted values. Given by 
                x' = F*x+v 
            Where
                - x'-> predicted value
                - x-> mean state vector containing position
                - v-> Process Noice
                - F-> State transition Matrix 
            State Transition Matrix will help use predict the future state of the robot
            based on the cinematic model. The model is Given by:
                x'_(t+1) = x_(t)+ vel*cos(theta)*dt
                y'_(t+1) = y_(t)+ vel*sin(theta)*dt
                theta_(t+1) = theta_(t) + dt*angular_vel
            This is the result of doign x' = F*x
            '''
            stateTransition = np.array([
                self.state[0] + self.velocity*np.cos(self.state[2])*self.dt,
                self.state[1] + self.velocity*np.sin(self.state[2])*self.dt,
                self.state[2] + self.angular*self.dt
            ])

            # Update de estimate state x' = F*x
            self.estimatedState = stateTransition
            print('/n')
            print("------------------------------------------------------------")
            print("1. Debug - EstimatedState: ", self.estimatedState, self.state)

            ''' Calculate the line model to be used in the uncertainty propagation
            Linalize the model, throught a Jacobian matrix, this is equivalent to the
            most significan coeficient in the taylor series expansion of the muti variable
            function around a given points.
            '''
            Jacobian = np.array([
                [1,0,-self.dt*self.velocity*np.sin(self.state[2])],
                [0,1,self.dt*self.velocity*np.cos(self.state[2])],
                [0,0,1]
            ])

            #Calculate the propagation of the uncertainty
            self.covariance = np.dot(np.dot(Jacobian, self.covariance),Jacobian.T) + self.Q

            print("2. Debug - Covariance: ", self.covariance)

            '''
            If no ArUco Marker was detected justr popagate the error of the odometry, however,
            if some marker was found propagate the error of the landmark measurment
            '''
            print("2.5 Debug - ArUco detected: ", self.ArUco_detected)

            if self.ArUco_detected == True:
                # Set the aproximate location of the ArUco Marker
                self.ArUcoLocation[0] = self.state[0] + self.tvec[0,0,2]
                self.ArUcoLocation[1] = self.state[1] + self.tvec[0,0,0]
                '''
                Estimate the ideal position of the land mark
                '''
                #delta_x = self.landMarks[0] - self.estimatedState[0]
                #delta_y = self.landMarks[1] - self.estimatedState[1]
                #p = np.power(delta_x, 2) + np.power(delta_y, 2)

                delta_x = self.ArUcoLocation[0]
                delta_y = self.ArUcoLocation[1]
                p = np.power(delta_x, 2) + np.power(delta_y, 2)


                # Calculate the observation model. The ideal measure for the LandMarks
                self.estimatedLM = np.array([
                    np.sqrt(p),
                    np.arctan2(delta_y,delta_x) - self.estimatedState[2]
                ])
                print("4. Debug - Ideal ArUco Location: ", self.estimatedLM)

                # Linearize the LandMarks observation model
                self.G[0,0] = -delta_x / np.sqrt(p)
                self.G[0,1] = -delta_y / np.sqrt(p)
                self.G[1,0] = delta_y / p 
                self.G[1,1] = -delta_x / p
                self.G[1,2] = -1        

                # Using the linearised model compute the measurment uncertainty propagation
                z = np.dot(np.dot(self.G, self.covariance),self.G.T) + self.R
                print("5. Debug - Uncertainty Propagation: ", self.estimatedState, self.state)

                # Calculate Kalman Gain
                self.kalmanGain = np.dot(np.dot(self.covariance, self.G.T), np.linalg.inv(z))
                print("6. Debug - kalmanGain: ", self.kalmanGain)

                self.execute = True

                self.update()
            else:
                print("The robot might be here only Odometry:")
                print(self.state)
                print("--------------------------------------")
        else:
            print("RobotState only odometry: ", self.state, "State: ", self.robotState)
            print("------------------------------------------------------------------")

    def update(self):
        # Calculate and update position of the robot using the real observation z
        estimatedPose = self.estimatedState + np.dot(self.kalmanGain,(self.closestMarker-self.estimatedLM))
        # Add a transition by 8cm in the x axis due to camera displacement

        print("7. Debug - The robot might be in postion with odom and ArUco:")
        print(self.state)

        # Calculate and update covariance
        self.covariance = np.dot((np.eye(3)-np.dot(self.kalmanGain,self.G)), self.covariance)
        print("8. Debug - Covariance Matrix: ", self.covariance)
        print("-----------------------------------------------")

    def superOdom(self):
        # Create the PoseWithCovariance message
        pose_cov_msg = PoseWithCovarianceStamped()
        
        pose_cov_msg.header.stamp = rospy.Time.now()
        pose_cov_msg.header.frame_id = "map"

        pose_cov_msg.pose.pose.position.x = self.state[0]  # Set the position of the ellipsoid
        pose_cov_msg.pose.pose.position.y = self.state[1]
        pose_cov_msg.pose.pose.position.z = self.state[2]
        
        # Set the orientation (quaternion)
        pose_cov_msg.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Default orientation
        
        # Set the covariance matrix

        #(BEGIN TEST)
        # Initialize a 6x6 matrix filled with zeros
        new_covariance = np.zeros((6, 6))

        # Copy the elements of the 3x3 covariance matrix
        new_covariance[:2, :2] = self.covariance[:2, :2]  # Covariances for x and y
        new_covariance[5, :2] = self.covariance[2,:2] # Covariance for thetha with respect to x and y
        new_covariance[:2, 5] = self.covariance[:2,2]
        new_covariance[5, 5] = self.covariance[2, 2]  # Variance for rotation in z

        # Set the covariances involving z and rotation in x, rotation in y, and rotation in z to zero
        new_covariance[2:5, 2:5] = 0.0
        #(END TEST)

        pose_cov_msg.pose.covariance = new_covariance.flatten().tolist()
        
        # Publish the PoseWithCovariance message
        self.odom_pub.publish(pose_cov_msg)

    def main(self):
        # Set the loop rate
        rate = rospy.Rate(10)

        # Set Robot Goals
        goals = np.array([
            [4.5, 0.0],
            [4.5, 4.5],
            [1.0 ,3.5],
            [0.0, 0.0]
        ])

        self.init_goal_set(goals)

        while not rospy.is_shutdown():
            self.predict()
            if self.execute & self.ArUco_detected: 
                # Update the state 
                self.update()
                self.execute = False
                self.ArUco_detected = False
            self.superOdom()

            rate.sleep()

if __name__ == '__main__':
    node = ExtendedKalmanFIlter()
    node.main()
    # Wait a sec
    cv2.waitKey(1)
    # Close all windows
    cv2.destroyAllWindows()