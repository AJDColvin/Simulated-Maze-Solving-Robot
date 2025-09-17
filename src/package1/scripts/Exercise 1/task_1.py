#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 23 16:45:12 2024

@author: ajdc21
"""

import rospy
from geometry_msgs.msg import Twist
from demo_programs.msg import prox_sensor
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

front_prox = 0
ranges = []
move_cmd = Twist()
maxVal = 0
log_message = ''

printed_turn = False


def callback(data):

    global move_cmd
    global maxVal
    global log_message
    
    global printed_turn
    
    temp = data.ranges
    ranges = temp[30:len(temp)-30] #cutoff Lidar readings of behind robot
    window = 25
    maxVal = len(ranges)*0.5#half the len of ranges as default
    
    if ranges:
        maxVal = ranges.index(max(ranges))

    if ranges[int(len(ranges)*0.5)]<0.3: #reverse briefly if object ahead
        move_cmd.linear.x = -0.5
        move_cmd.angular.z = -0.3
        rospy.loginfo('Bot reversed')
        log_message = 'Reversing'
        

    elif maxVal > 0.5*len(ranges) + window:
        move_cmd.linear.x = 0.1
        move_cmd.angular.z = -0.3
        rospy.loginfo('Bot turned left')
        log_message = 'Turning gradual left'
        

    elif maxVal < 0.5*len(ranges) - window:
        move_cmd.linear.x = 0.1
        move_cmd.angular.z = 0.3
        rospy.loginfo('Bot turned right')
        log_message = 'Turning gradual right'
        
        
    else:
        move_cmd.linear.x = 0.5 #move forward
        move_cmd.angular.z = 0.0
        log_message = 'Moving forward'
        
        if printed_turn == True:
            printed_turn = False

    
def move():
    # Publishers
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/log_messages', String, queue_size=10)
    

    # Subscriber
    rospy.Subscriber('/hokuyo', LaserScan, callback)
    
    # Initialise Node
    rospy.init_node('control_bot', anonymous=True)
    
    rate = rospy.Rate(10) # 10hz
    

    
    #Rotate right if index[max(ranges)] > len(ranges)/2 + window
    #Rotate left if index[max(ranges)] < len(ranges)/2 - window
    
    
    while not rospy.is_shutdown():
 
        rospy.loginfo(move_cmd)
        rospy.loginfo(maxVal)
        pub.publish(move_cmd)
        pub2.publish(log_message)
    
        rate.sleep()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
