#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO  
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

class BittlePlannerSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            1)
        self.bridge = CvBridge()
        self.model = YOLO('/home/rithwik/bittle/autonomous-bittle/vision-based-command/best.pt')


        self.detection_publisher = DetectionPublisher()

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Perform object detection
        results = self.model([current_frame])  
        
        annotated_frame = results[0].plot()

        cv2.imshow("YOLOv8 Detection with bbox", annotated_frame)
        cv2.waitKey(1)
        
        # Publish detection results
        if len(results) > 0:
            result_list = (results[0].boxes.cls).cpu().tolist()
            xywhn_list = (results[0].boxes.xywhn).cpu().tolist()
        
        detection_info = [{'results': result_list, 'xywhn_list': xywhn_list}]
        
        self.detection_publisher.publish_detection_info(detection_info)
    
    # def pheromone_callback(self, msg):
    #     '''
    #     Process camera feed for bead detection and path planning
    #     Implement logic to detect green and purple beads
    #     Implement path planning algorithm based on bead detection
    #     '''

    #     if self.cv2_img is not None:
            
    #         #object detection using YOLO
    #         results = self.model(self.cv2_img)
            
    #         #Initialize lists
    #         acorns = [] #Found Acorns
    #         green_pheromones = [] #Happy State
    #         black_pheromones = [] #Search State

    #         #looping through detected objects
    #         if len(results) > 0:
    #             result_list = (results[0].boxes.cls).cpu().tolist()
    #             xywhn_list = (results[0].boxes.xywhn).cpu().tolist()
    #             for i in range(len(xywhn_list)):
    #                 x, y, w, h, _ = xywhn_list[i]
    #                 if result_list[i] == 0.0:  # Green Pheromone
    #                     green_pheromones.append((x, y, w, h))
    #                 elif result_list[i] == 1.0:  # Black Pheromone
    #                     black_pheromones.append((x, y, w, h))
    #                 elif result_list[i] == 2.0:  # Acorn
    #                     acorns.append((x, y, w, h))
            
    #         #Move towards detected pheromones
    #         if acorns:
    #             self.move_to_pheromones(acorns[0])
    #         elif green_pheromones:
    #             self.move_to_pheromones(green_pheromones[0])
    #         elif black_pheromones:
    #             self.move_to_pheromones(black_pheromones[0])
    #         else:
    #             self.rotate_bittle()
        
    # def move_to_pheromones(self, pheromone, msg):
    #     #assuming
    #     x_pheromone = pheromone[0]
    #     y_pheromone = pheromone[1]

    #     #make narrow window
    #     x_distance = self.bittle_center_x - x_pheromone
    #     y_distance = self.bittle_center_y - y_pheromone

    #     self.x_boundary_left = self.bittle_center_x - 50
    #     self.x_boundary_right = self.bittle_center_x + 50

    #     angle = np.arctan(x_distance/y_distance)

    #     while x_pheromone < self.x_boundary_left or x_pheromone > self.x_boundary_right:
    #         while angle > 0.1: #turn right
    #             try:
    #                 self.dir = 3
                    
    #             except CvBridgeError as e:
    #                 self.get_logger().error(str(e))
    #         while angle < -0.1: #turn left
    #             try:
    #                 self.dir = 4 
    #             except CvBridgeError as e:
    #                 self.get_logger().error(str(e))
    #         self.dir = 0
        
    #     #once the pheromone is within the narrow window, move forward towards it
    #     while x_pheromone >= self.x_boundary_left or x_pheromone <= self.x_boundary_right:
    #         buttons = msg.buttons
    #         axes = msg.axes
    #         try:
    #             self.dir = 1
    #             # time.sleep(1)
    #             # axes[1] == 0
            
    #         except CvBridgeError as e:
    #             self.get_logger().error(str(e))
            
    #         self.dir = 0
        
    # def rotate_bittle(self, msg):
    #     '''
    #     If bot doesnt find any sort of pheromone, rotate it by 10 deg until it finds a pheromone or acorn
    #     '''
    #     buttons = msg.buttons
    #     axes = msg.axes
    #     try:
    #         axes[0] = -1 
    #         time.sleep(1)
    #     except CvBridgeError as e:
    #         self.get_logger().error(str(e))

    # def boundary_callback (self, msg):
        # '''
        # Process camera feed to detect the blue tape boundary
        # Calculate angle between tape line and boundary line to position the bot perpendicular to the tape
        # '''
        # try:
        #     cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
        #     # Convert image to grayscale
        #     gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
        #     # Detect edges using Canny edge detection
        #     edges = cv2.Canny(gray_image, 50, 150, apertureSize=3)
            
        #     # Find lines using Hough Line Transform
        #     lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)
            
        #     # Process detected lines
        #     if lines is not None:
        #         for line in lines:
        #             x1, y1, x2, y2 = line[0]
        #             cv2.line(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Draw detected lines
                
        #         # Calculate angle between detected line and bottom boundary of the camera frame
        #         tape_angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                
        #         # Adjust bot's orientation based on tape angle
        #         bot_angle = -tape_angle + 90  # Assuming 90 degrees is perpendicular to the bottom boundary
                

        # except CvBridgeError as e:
        #     self.get_logger().error(str(e))


class DetectionPublisher(Node):
    def __init__(self):
        super().__init__('detection_publisher')
        self.publisher = self.create_publisher(Detection, 'detection_topic', 10)
        
    def publish_detection_info(self, detection):
        msg = Detection()
        msg.results = [int(result) for result in detection[0]['results']]
        # Initialize an empty list to hold the flattened and converted values
        flattened_xywhn_list = []

        # Iterate through each sublist in 'xywhn_list'
        for sublist in detection[0]['xywhn_list']:
            # Check if the item is a list (to handle nested lists)
            if isinstance(sublist, list):
                # Iterate through each item in the sublist
                for item in sublist:
                    # Convert each item to float and append to the flattened list
                    flattened_xywhn_list.append(float(item))
            else:
                # If the item is not a list, convert and append directly
                flattened_xywhn_list.append(float(sublist))

        # Assign the flattened list of floats to 'msg.xywhn_list'
        msg.xywhn_list = flattened_xywhn_list
        self.publisher.publish(msg)
        self.get_logger().info('Publishing detection')


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = BittlePlannerSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
