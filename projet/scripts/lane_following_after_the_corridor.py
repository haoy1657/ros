#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#This program is used for line following for the path after the corridor only. It also uses a camera and laser scan data to track path. We have thus reused the code developed in the previous file (challenge12.py) to write this one.

#However, there are a few changes here, where self.Kp is defined as 0.0032 and h is set to 50, n is 50 and m is 50; to make the robot more responsive during a sudden deviation from the trajectory.If we don't change these values, the robot will not follow the line and will go out of the indicated zone.

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import math

class PathTracker_after_corridor():
    def __init__(self):    
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.ready_to_move = False

        self.Kp = 0.032
        self.Ki = 0
        self.Kd = 0
        self.integral = 0    
        self.prev_error = 0
        self.first_run = True

        self.mode = 'normal'

        self.cv_image = None
        self.laser_data = None

    def image_callback(self, data):
        try:
            print(self.Kp)
            self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

    def laser_callback(self, data):
        self.laser_data = data

if __name__ == '__main__':
    rospy.init_node('PathTracker_after_corridor', anonymous=True)
    path_tracker = PathTracker_after_corridor()

    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        if path_tracker.cv_image is not None and path_tracker.laser_data is not None:
            path_tracker.Kp=0.13
            height, width = path_tracker.cv_image.shape[:2]
            h = 50
            n = 50
            m = 50

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

            obstacle_detected = False
            # Scan the 30-degree range in front of the robot
            for i in range(-58, 59):
                angle = math.radians(i)
                index = int((angle - path_tracker.laser_data.angle_min) / path_tracker.laser_data.angle_increment)
                if 0.1 <= path_tracker.laser_data.ranges[index] <= 0.2:
                    print(path_tracker.laser_data.ranges[index])
                    obstacle_detected = True
                    break
            
     
                    
            
            

            path_tracker.cmd_vel_pub.publish(twist)

            rate.sleep()


    # Wait for a keypress or exit
    key = cv2.waitKey(1)
    if key == ord('q') or key == 27:
        rospy.signal_shutdown("User terminated the program.")
        cv2.destroyAllWindows()()
