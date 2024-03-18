import rclpy
from rclpy.node import Node
import serial
import struct
import sys
import time
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from bittle_msgs.msg import Detection

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time
from ultralytics import YOLO
import numpy as np
import serial
from bittle_msgs.msg import Detection


def boundary_callback(self, msg):

    self.boundary_threshold_x = 50

    cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
    # Convert image to grayscale
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
    # Detect edges using Canny edge detection
    edges = cv2.Canny(gray_image, 50, 150, apertureSize=3)
    
    # Find lines using Hough Line Transform
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)
    
    # Process detected lines
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Draw detected lines
        
        # Calculate angle between detected line and bottom boundary of the camera frame
        tape_angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
        
        # Adjust bot's orientation based on tape angle
        bot_angle = -tape_angle + 90  # Assuming 90 degrees is perpendicular to the bottom boundary
    
    while abs(bot_angle - 90) > 10:
        self.dir = 3

    self.dir = 0

    while x1 > self.boundary_threshold_x:
        self.dir = 1
    
    self.dir = 0

    self.dir = 3
    time.sleep(5)
    self.dir = 0

    



    


    