#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import cv2, PIL
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
from nav_msgs.msg import Odometry
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
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.odom_sub = rospy.Subscriber('/odom_gazebo', Odometry, self.odom_callback)

        #------------Initialize EKF variables-------------------
        # Set landmarks array
        self.landMarks = np.array([1.0,0.0])

        # Set the Aruco measurment
        self.ArUcoLocation = np.zeros(2)
        '''
        Assume sample timining dt, angular and lineal velocity as constants
        '''
        self.velocity = 0.55 # In m/s
        self.angular = 1.0 # In rad/s
        self.dt = 0.1 # In s

        '''
        Initial state of the robot starts at [0,0,0]
        The state vector represents the current state of the robot and include has the form:
            [x,y,theta] 
        not that velocity is asume to be constant
        '''
        self.state = np.zeros(3)
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
        self.covariance = np.zeros(3) 

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

            #Update robot State variables
            self.state[0] = x
            self.state[1] = y
            self.state[2] = theta

        except Exception as e:
            rospy.logerr(e)

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
            self.ArUco_detected = True
            for i in range(0, len(ids)): #Do for each ArUco marcker
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

                '''
                Tvec is the vector from the camera frame to the center of the ArUco marker
                Rvec is the rotation of the marker viewed from the camera frame
                Since Tvec and Rvec are expresed in the camera frame which is described as follows
                    - z-> is defined as the optical axis of the camera +Z is in front of the camera
                    - x -> +x points to the right
                    - y -> +y points down
                z in the camera frame is x in the base frame as well y-> z and x->y
                '''
                # Set the aproximate location of the ArUco Marker
                self.ArUcoLocation[0] = self.state[0] + tvec[0,0,2]
                self.ArUcoLocation[1] = self.state[1] + tvec[0,0,0]
                # Print rvecs and tvecs
                
                #Show result
                cv2.imshow("ArUco marker detector", frame)
        else:
            self.ArUco_detected = False
            print("Warning: No ArUco markers found")

    def predict(self):
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

        '''
        If no ArUco Marker was detected justr popagate the error of the odometry, however,
        if some marker was found propagate the error of the landmark measurment
        '''
        if self.ArUco_detected:
            '''
            Estimate the ideal position of the land mark
            '''
            delta_x = self.landMarks[0] - self.estimatedState[0]
            delta_y = self.landMarks[1] - self.estimatedState[1]
            
            p = np.power(delta_x, 2) + np.power(delta_y, 2)

            # Calculate the observation model. The ideal measure for the LandMarks
            self.estimatedLM = np.array([
                np.sqrt(p),
                np.arctan2(delta_y,delta_x) - self.estimatedState[2]
            ])

            # Linearize the LandMarks observation model
            self.G[0,0] = -delta_x / np.sqrt(p)
            self.G[0,1] = -delta_y / np.sqrt(p)
            self.G[1,0] = delta_y / p 
            self.G[1,1] = -delta_x / p
            self.G[1,2] = -1        

            #Using the linearised model compute the measurment uncertainty propaation
            z = np.dot(np.dot(self.G, self.covariance),self.G.T) + self.R

            # Calculate Kalman Gain
            self.kalmanGain = np.dot(np.dot(self.covariance, self.G.T), np.linalg.inv(z))

            self.execute = True
        else:
            print("The robot might be here:")
            print(self.state)

    def update(self):
        # Calculate and update position of the robot using the real observation z
        self.state = self.estimatedState + np.dot(self.kalmanGain,(self.ArUcoLocation-self.estimatedLM))
        print("The robot might be in postion:")
        print(self.state)

        # Calculate and update covariance
        self.covariance = np.dot((np.eye(3)-np.dot(self.kalmanGain,self.G)), self.covariance)

    def drawElipsoide(self):
            # Grafica la elipsoide de confianza
            fig = plt.figure()
            ax = fig.add_subplot(111)    # Extrae los valores de la covarianza
            cov_values, cov_vectors = np.linalg.eig(self.covariance[:2, :2])    # Calcula los semiejes de la elipsoide
            a, b = np.sqrt(cov_values)    # Genera los puntos para graficar la elipsoide
            u = np.linspace(0, 2 * np.pi, 100)
            x = a * np.cos(u)
            y = b * np.sin(u)    # Rota y traslada la elipsoide según el estado actual del robot
            theta = self.state[2]
            c, s = np.cos(theta), np.sin(theta)
            rotation_matrix = np.array([[c, -s], [s, c]])
            ellipse_points = np.dot(rotation_matrix, np.array([x, y]))
            ellipse_points[0, :] += self.state[0]
            ellipse_points[1, :] += self.state[1]    # Grafica la proyección de la elipsoide en el plano XY
            ax.plot(ellipse_points[0, :], ellipse_points[1, :], color='b', alpha=0.3)    # Configura el aspecto de la gráfica
            ax.set_aspect('equal')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_title('Proyección de la elipsoide de confianza')    # Muestra la gráfica
            plt.show()

    def main(self):
        # Set the loop rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.predict()
            if self.execute:
                self.update()
                self.execute = False
            #self.drawElipsoide()
            rate.sleep()

if __name__ == '__main__':
    node = ExtendedKalmanFIlter()
    node.main()
    # Wait a sec
    cv2.waitKey(1)
    # Close all windows
    cv2.destroyAllWindows()