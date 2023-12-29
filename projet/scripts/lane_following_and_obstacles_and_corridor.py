#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This program uses a camera and laser scan data to track path; It detects  obstacles and corridors, and adjusts the robot's movement accordingly.

# Importation of necessary packages and modules
import rospy
from sensor_msgs.msg import Image # to handle sensor data messages
from cv_bridge import CvBridge, CvBridgeError # to convert ROS images to OpenCV format 
from geometry_msgs.msg import Twist # to define the robot's movement commands
from sensor_msgs.msg import LaserScan
import cv2 # for computer vision tasks
import numpy as np
import math

class PathTracker():
    # initialisation of the class attributes	
    def __init__(self):    
        self.bridge = CvBridge() # initializes the CvBridge instance for converting ROS images to OpenCV format.
        # creation of subscribers to the ROS topics '/camera/image' and '/scan' to receive image and laser scan data respectively, and set their corresponding callback functions to be called upon receiving new messages.
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        # creation of a publisher to the ROS topic '/cmd_vel' with message type "Twist" and a queue size of 1. It allows the program to send movement commands to the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.ready_to_move = False #Initializes a boolean attribute to indicate if the robot is ready to move or not

        self.Kp = 0.05 #Initializes the proportional gain of the path tracking algorithm
        self.Ki = 0 # Initializes the integral gain of the path tracking algorithm
        self.Kd = 0.01 #Initializes the derivative gain of the path tracking algorithm
        self.integral = 0 #Initializes the integral term for the PID controller   
        self.prev_error = 0 # Initializes the error value from the previous cycle for the PID controller
        self.first_run = True # Initializes a boolean attribute to indicate if it is the first cycle of the PID controller
        
        self.mode = 'normal' #Initializes the mode of operation for the robot
        self.cv_image = None #Initializes the variable for storing the image data received from the camera
        self.laser_data = None #Initializes the variable for storing the laser scan data received from the laser sensor



    #This function is a callback for the image subscriber. It converts the received image data from ROS format to OpenCV format using CvBridge. If any errors occur during the conversion, the function prints the error message and stop further execution 
    def image_callback(self, data):
        try:
            
            self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
            
            
                    
    #This function is a callback for the laser scanner subscriber
    def laser_callback(self, data):
        self.laser_data = data



if __name__ == '__main__':
    rospy.init_node('path_tracker', anonymous=True) #Initializes the ROS node with the name 'path_tracker' 
    path_tracker = PathTracker() #instance of the PathTracker class

    rate = rospy.Rate(10) # frequency of 10 Hz, control the loop rate and ensure that the node runs at a consistent frequency.

    while not rospy.is_shutdown(): #starts a loop that runs continuously until the rospy shutdown signal is received
        if path_tracker.cv_image is not None and path_tracker.laser_data is not None: #check if the robot's camera and laser sensors are both available
        

            #used to track whether an obstacle or a corridor has been detected
            obstacle_detected = False
            corridor_detected=False
            
            
            
            # obstacle detection within a certain range using the robot's laser sensor
            for i in range(-58, 59):
                angle = math.radians(i)
                index = int((angle - path_tracker.laser_data.angle_min) / path_tracker.laser_data.angle_increment) # conversion of a range of angles and convert them to index values that correspond to the laser readings
                if 0.1 <= path_tracker.laser_data.ranges[index] <= 0.2:
                    #print(path_tracker.laser_data.ranges[index])
                    obstacle_detected = True #variable set to True if the obstacle is detected within a certain range
                    break
                    
                    
            # Corridor detection using the robot's laser sensor
            # The left_wall and right_wall variables are determined by taking the minimum distance measurement within certain ranges of the laser data
            left_wall  = min(path_tracker.laser_data.ranges[85:95])
            right_wall = min(path_tracker.laser_data.ranges[265:275])
            #The front_wall variable is determined by taking the minimum distance measurement within two ranges of the laser data, one from the front right and one from the front left of the robot. This determines the distance to any obstacles in front of the robot
            front_wall = min(path_tracker.laser_data.ranges[355:] + path_tracker.laser_data.ranges[:6])

            if left_wall < 0.30 and right_wall < 0.30 and front_wall>0.35 and front_wall<2:
                #If these conditions are met, then corridor_detected is set to True, indicating that the robot is currently in a corridor.
                corridor_detected=True
            
            if obstacle_detected and not corridor_detected :
                height, width = path_tracker.cv_image.shape[:2] #height and width of the current image frame captured by the robot's camera
                #These following variables are used to determine the size and location of a region of interest in the image (ROI)
                h = 70
                n = 75
                m = 40

                region = path_tracker.cv_image[height - h - 1:height - 1, m:m + n] #creates an ROI in the image by selecting a subset of the pixels in the original image
                gray_region = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY) #converts the color image in the ROI to grayscale
                _, binary_region = cv2.threshold(gray_region, 128, 255, cv2.THRESH_BINARY) #applies a threshold to the grayscale image to create a binary image

                moments = cv2.moments(binary_region) #calculates the moments of the binary image, which are used to determine the centroid of the object in the ROI
                if moments['m00'] != 0:
                    #These two following lines calculate the centroid of the object in the ROI 
                    cx = int(moments['m10'] / moments['m00'])
                    cy = int(moments['m01'] / moments['m00'])
                    # draw a circle around the centroid of the object in the ROI to highlight its location
                    cv2.circle(region, (cx, cy), 5, (0, 0, 255), -1)
                    

                    error = cx - region.shape[1] / 2 #computes the distance between the center of the ROI and the current position of the robot in the image
                    
                    # this following block of code implements a simple proportional-integral-derivative (PID) controller to steer the robot towards the center of the ROI in the image
                    if not path_tracker.first_run:
                        derivative = error - path_tracker.prev_error
                        path_tracker.integral += error
                        control = path_tracker.Kp * error + path_tracker.Ki * path_tracker.integral + path_tracker.Kd * derivative
                    else:
                        control=0
                        path_tracker.first_run = False

                    path_tracker.prev_error = error
                    cv2.namedWindow("Image window")
                    cv2.imshow("Image window", region)
                    cv2.waitKey(1)

                # these following lines set up a Twist message to control the robot's motion in normal mode
                twist = Twist()
                twist.linear.x = 0.05
                twist.angular.z = -float(control)
                
                
                # these following lines switche the robot to avoidance mode 
                path_tracker.Kp = 0.032
                print("Avoidance mode")
                twist.linear.x = 0.03
                # these three following variables to represent the minimum distance to obstacles detected by the robot's laser sensor on the left, right, and front sides
                min_distance_left = float('inf')
                min_distance_right = float('inf')
                min_distance_front = float('inf')


		# This following code block loops through the laser data to determine the minimum distance readings to the left, right, and front of the robot
                for i in range(len(path_tracker.laser_data.ranges)):
                    angle = path_tracker.laser_data.angle_min + i * path_tracker.laser_data.angle_increment  #calculates the angle of the current laser reading
                    if 0.1 <= path_tracker.laser_data.ranges[i] <= 10.0:#checks if the current laser reading is within the specified range (0.1 to 10.0 meters)
                    #Depending on the value of angle, the current reading is compared with either min_distance_left or min_distance_right and the minimum value is stored
                        if 0 <= angle < math.pi:
                            min_distance_left = min(min_distance_left, path_tracker.laser_data.ranges[i])
                        else:
                            min_distance_right = min(min_distance_right, path_tracker.laser_data.ranges[i])

                        # Check for obstacles in front of the robot
                        if -15 <= angle * 180 / math.pi <= 15:
                            min_distance_front = min(min_distance_front, path_tracker.laser_data.ranges[i])
                print(min_distance_left,min_distance_right,min_distance_front)
                
                if min_distance_front <= 0.25: #If the minimum distance in front is less than or equal to 0.25 meters, the robot turns right to avoid the obstacle
                    # Obstacle in front, turn right
                    print("obstacle in front, turn right")
                    twist.angular.z = -1
                    
                elif min_distance_left > min_distance_right:#If the minimum distance on the left is greater than the minimum distance on the right, the robot turns left
                    # Turn left
                    print("turn left")
                    twist.angular.z = 1
                    
                else: #Otherwise, the robot turns right
                    # Turn right
                    print("turn right")
                    twist.angular.z = -1
                    
                    
                    
	    #This following code block sets the robot to operate in corridor mode if the variable corridor_detected is True	
            elif corridor_detected :          
                    print("corridor mode")
                    
                    twist.linear.x = 0.03
                    min_distance_left = float('inf')
                    min_distance_right = float('inf')
                    min_distance_front = float('inf')

                    for i in range(len(path_tracker.laser_data.ranges)):
                        angle = path_tracker.laser_data.angle_min + i * path_tracker.laser_data.angle_increment
                        if 0.1 <= path_tracker.laser_data.ranges[i] <= 10.0:
                            if 0 <= angle < math.pi:
                                min_distance_left = min(min_distance_left, path_tracker.laser_data.ranges[i])
                            else:
                                min_distance_right = min(min_distance_right, path_tracker.laser_data.ranges[i])
                            
                         

                            # Check for obstacles in front, left and right directions using the laser range data.  It sets variables min_distance_front, min_distance_left, and min_distance_right to the minimum distance observed in each direction
                            if -15 <= angle * 180 / math.pi <= 15:
                                min_distance_front = min(min_distance_front, path_tracker.laser_data.ranges[i])
                    print(min_distance_left,min_distance_right,min_distance_front)
           
                    if min_distance_front <= 0.6 and min_distance_left > min_distance_right :
                        print("corridor in front and min_distance_left > min_distance_right, turn left ")
                        twist.angular.z = 0.07
                        
                    elif min_distance_front <= 0.5 and min_distance_left < min_distance_right:
                        print("corridor in front and min_distance_left < min_distance_right, turn right ")
                        twist.angular.z = -0.07
                        
                    elif  min_distance_front >= 0.5 and min_distance_left > min_distance_right:
                        # Turn left
                        print("turn left")
                        twist.angular.z = 0.025
                    
                    else:
                        # Turn right
                        print("turn right")
                        twist.angular.z = -0.025
                        
                        
                        
            #The else code block that follows the elif corridor_detected block is executed when the variable corridor_detected evaluates to False. This code block is used to operate the robot in normal mode, where it follows lines on the floor using a camera and image processing algorithms            
            else:
                path_tracker.Kp=0.032
                path_tracker.Kd=0.05
                height, width = path_tracker.cv_image.shape[:2]
                h = 70
                n = 75
                m = 40

                region = path_tracker.cv_image[height - h - 1:height - 1, m:m + n]
                gray_region = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
                _, binary_region = cv2.threshold(gray_region, 128, 255, cv2.THRESH_BINARY)

                moments = cv2.moments(binary_region)
                if moments['m00'] != 0:
                    cx = int(moments['m10'] / moments['m00'])
                    cy = int(moments['m01'] / moments['m00'])
                    cv2.circle(region, (cx, cy), 5, (0, 0, 255), -1)
                    print("mode normal")

                    error = cx - region.shape[1] / 2
                    if not path_tracker.first_run:
                        derivative = error - path_tracker.prev_error
                        path_tracker.integral += error
                        control = path_tracker.Kp * error + path_tracker.Ki * path_tracker.integral + path_tracker.Kd * derivative
                    else:
                        control=0
                        path_tracker.first_run = False

                    path_tracker.prev_error = error
                    cv2.namedWindow("Image window")
                    cv2.imshow("Image window", region)
                    cv2.waitKey(1)

                # Normal mode	
                twist = Twist()
                twist.linear.x = 0.05
                twist.angular.z = -float(control)
            
                    
            
            

            path_tracker.cmd_vel_pub.publish(twist) #publishes the computed twist message to the cmd_vel topic

            rate.sleep() #keep the loop running at a constant rate

