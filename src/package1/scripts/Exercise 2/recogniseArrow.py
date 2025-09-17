#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  6 16:31:31 2024

@author: ajdc21
"""
import rospy
from geometry_msgs.msg import Twist

# Load required packages
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model 
from PIL import Image
import glob
import os
import matplotlib.pyplot as plt

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"


class RecogniseArrowsNode:
    def __init__(self):
        
        # List of arrow classes
        self.namesList = ['up', 'down', 'left', 'right']
        
        # Folder path for testing images
        self.imageFolderTestingPath = r'./Database_arrows/validation'
        self.imageTestingPath = []
        
        # Full path to testing images
        for i in range(len(self.namesList)):
            self.testingLoad = self.imageFolderTestingPath + '/' + self.namesList[i] + '/*.jpg'
            self.imageTestingPath = self.imageTestingPath + glob.glob(self.testingLoad)
        
        # Print number of testing images
        print(f"Number of testing images: {len(self.imageTestingPath)}")
        
        # Load the pre-trained model
        self.model_path = "CNN_Model.keras"
        self.model = load_model(self.model_path)

        # Publishers
        self.pub1 = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #Initialise node
        rospy.init_node('RecogniseArrows', anonymous=True)

        self.move_cmd = Twist()
        
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.computeArrow()
            self.rate.sleep()
    

    def computeArrow(self):
        #Prompt user to continue or exit
        user_input = input("Press enter for next arrow, or type 'exit' to quit: ")
        if user_input.lower() != 'exit':
            # Randomly select an image from the test dataset
            random_index = np.random.randint(len(self.imageTestingPath))
            random_image_path = self.imageTestingPath[random_index]
        
            # Get the actual class label
            actual_label_index = random_index // (len(self.imageTestingPath) // len(self.namesList))
            actual_label = self.namesList[actual_label_index]
        
            # Load and preprocess the image
            updateImageSize = [128, 128]
            tempImg = Image.open(random_image_path).convert('L')
            tempImg.thumbnail(updateImageSize, Image.ANTIALIAS)
            [imWidth, imHeight] = tempImg.size
            input_image = np.zeros((1, imHeight, imWidth, 1))
            input_image[0, :, :, 0] = np.array(tempImg, 'f')
            
        
            # Make a prediction
            prediction = self.model.predict(input_image)
            predicted_label_index = np.argmax(prediction)
            predicted_label = self.namesList[predicted_label_index]
        
            # Display the actual and predicted results
            plt.imshow(tempImg, cmap='gray')
            plt.title(f"I think the arrow is: {predicted_label} (Actual: {actual_label})")
            plt.axis('off')
            plt.show(block=False)
            plt.pause(0.001)
        
            # Turn arrow prediction into move command
            if predicted_label == "up":
                self.move_cmd.linear.x = 0.3
                self.move_cmd.angular.z = 0.0
                
            elif predicted_label == "down":
                self.move_cmd.linear.x = -0.3
                self.move_cmd.angular.z = 0.0
                
            elif predicted_label == "left":
                self.move_cmd.linear.x = 0.3
                self.move_cmd.angular.z = -0.37
                
            elif predicted_label == "right":
                self.move_cmd.linear.x = 0.3
                self.move_cmd.angular.z = 0.37
            
            
            self.pub1.publish(self.move_cmd)
            rospy.sleep(3.0)
            
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.pub1.publish(self.move_cmd)
        
        else:
            print("----------End of programme----------")
            rospy.signal_shutdown("Finished work")
            
        
         
            
    
        

        

if __name__ == '__main__':
    try:
        node = RecogniseArrowsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



