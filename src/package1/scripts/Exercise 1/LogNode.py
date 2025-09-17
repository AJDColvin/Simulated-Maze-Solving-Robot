#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 18 10:18:46 2024

@author: ajdc21
"""

import rospy
import time 
from std_msgs.msg import String

class LoggingNode:
    def __init__(self):
        
        #logfile name
        logfile = rospy.get_param('logfile', 'default_logfile.txt')
        
        
        #Initialise node
        rospy.init_node('LoggingNode', anonymous=True)
        
        #Subscribe to log_messages topic
        self.sub1 = rospy.Subscriber('/log_messages', String, self.callback)
        
        #Time variables for log files
        self.start_time = time.time()
        self.curr_time = 0
        
        #Hold previous message
        self.prev_data = ""
        
        #Log file 
        self.filepath = "/rosdata/ros_ws/src/package1/logs/" + logfile
        with open(self.filepath, "w") as self.f:
            self.f.write('LOG FILE:\n\n')
        
        
    def callback(self, data):
         
        if data.data != self.prev_data:
            self.f = open(self.filepath, "a")
            self.curr_time = str(round(time.time() - self.start_time, 2))
            self.f.write(self.curr_time + ": " + data.data + "\n")
            rospy.loginfo(data.data)
            self.prev_data = data.data
            self.f.close()
        

if __name__ == '__main__':
    try:
        node = LoggingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
    
