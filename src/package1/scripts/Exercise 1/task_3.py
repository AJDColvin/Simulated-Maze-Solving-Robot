#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 15 11:10:25 2024

@author: ajdc21
"""

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from demo_programs.msg import prox_sensor
from demo_programs.msg import line_sensor
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String 

class Task3Node:
    def __init__(self):

        # Subscribers
        self.sub1 = rospy.Subscriber('/cop/prox_sensors', prox_sensor, self.proxCallback)
        self.sub2 = rospy.Subscriber('/hokuyo', LaserScan, self.lidarCallback)
        self.sub3 = rospy.Subscriber('/cop/pose', Pose, self.poseCallback)
        self.sub4 = rospy.Subscriber('/cop/line_sensors', line_sensor, self.lineCallback)
        
        # Publishers
        self.pub1 = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub2 = rospy.Publisher('/log_messages', String, queue_size=10)
        
        #Initialise node
        rospy.init_node('task3', anonymous=True)

        #Variable to hold log messages
        self.log_message = ""

        #Variables for Lidar callback
        self.ranges = []
        self.window = 25 #Window for central lidar window
        self.maxVal = len(self.ranges)*0.5 #half the len of ranges as default
        
        #Variables for Prox callback 
        self.threshold = 0.2
        
        #Variables to check orientation is always forwards
        self.facing_back_filepath = "/rosdata/ros_ws/src/package1/log_files.txt"
        self.facing_back_r = False
        
        self.move_cmd = Twist()

        
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.computeMove()
            self.rate.sleep()

    def lidarCallback(self, data):        
        self.temp = data.ranges
        self.ranges = self.temp[50:len(self.temp)-50] #cutoff Lidar readings of behind robot
        
        #Find the longest Lidar path
        if self.ranges:
            self.maxVal = self.ranges.index(max(self.ranges))
        

    def proxCallback(self, data):   
        self.front_l = data.prox_front_left
        self.front_ll = data.prox_front_left_left
        self.front_r = data.prox_front_right
        self.front_rr = data.prox_front_right_right
    
    def poseCallback(self, data):
        self.orient = data.orientation.z
    
    def lineCallback(self, data):
        self.left_line = data.line_left
        self.middle_line = data.line_middle
        self.right_line = data.line_right
        

    def computeMove(self):
        
        #Check Orientation is not backwards
        if self.orient <  0.5 and self.orient > 0:
            self.facing_back_r = True
            self.facing_back_l = False
        if self.orient <  0.0 and self.orient > -0.5:
            self.facing_back_r = False
            self.facing_back_l = True
        else:
            self.facing_back_l = False
            self.facing_back_r = False
        
        if self.left_line == False and self.right_line == False and self.middle_line == False:
            #Check prox sensors triggered
            # if (self.front_l != 0 or self.front_ll !=0) and self.front_r == 0 and not self.facing_back_r:
            if (self.front_ll > 0 and self.front_ll < 0.25) and self.front_r == 0 and not self.facing_back_r: 
                self.move_cmd.angular.z = 0.5
                self.move_cmd.linear.x = 0.0
                rospy.loginfo("Turning sharp right")
                self.log_message = ("Turning sharp right")
                self.pub1.publish(self.move_cmd)
                rospy.sleep(0.5)
                
            # elif (self.front_r != 0 or self.front_rr !=0) and self.front_l ==0 and not self.facing_back_l:
            elif (self.front_rr > 0 and self.front_rr < 0.25) and self.front_l ==0 and not self.facing_back_l:
                self.move_cmd.angular.z = -0.5
                self.move_cmd.linear.x = 0.0
                rospy.loginfo("Turning sharp left")
                self.log_message = ("Turning sharp left")
                self.pub1.publish(self.move_cmd)
                rospy.sleep(0.5)
            
            #Check if longest Lidar path not infront
            elif self.maxVal > 0.5*len(self.ranges) + self.window and not self.facing_back_l:
                self.move_cmd.linear.x = 0.1
                self.move_cmd.angular.z = -0.3
                rospy.loginfo('Turning gradual left')
                self.log_message = ("Turning gradual left")
                
            elif self.maxVal < 0.5*len(self.ranges) - self.window and not self.facing_back_r:
                self.move_cmd.linear.x = 0.1
                self.move_cmd.angular.z = 0.3
                rospy.loginfo('Turning gradual right')
                self.log_message = ("Turning gradual right")
                
            else:
                self.move_cmd.linear.x = 0.7 #move forward
                self.move_cmd.angular.z = 0.0
                rospy.loginfo('Moving forward')
                self.log_message = ("Moving forward")
    
            #self.move_cmd.angular.z = 0.2
            #self.move_cmd.linear.x = 0.0
            self.pub1.publish(self.move_cmd)
            self.pub2.publish(self.log_message)
        else:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.pub1.publish(self.move_cmd)
            self.log_message = ("--------END OF MAZE----------")
            self.pub2.publish(self.log_message)
            rospy.loginfo("----------END OF MAZE----------")
            self.pub1.unregister()
            self.pub2.unregister()
            rospy.signal_shutdown("Finished work")
            rospy.sleep(3.0)
        

if __name__ == '__main__':
    try:
        node = Task3Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


