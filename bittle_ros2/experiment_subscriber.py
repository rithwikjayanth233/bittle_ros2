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

print('test1')

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
        confidence_region = [(0.25, 1), (0.25, 0.5), (0, 0.3), (0.75, 1), (0.75, 0.5), (1, 0.3)]
        # for i in range(len(confidence_region) - 1):
        #     cv2.line(annotated_frame, tuple(np.array(confidence_region[i]) * np.array(annotated_frame.shape[:2][::-1])), 
        #              tuple(np.array(confidence_region[i+1]) * np.array(annotated_frame.shape[:2][::-1])), (0, 255, 0), 1)
        # cv2.line(annotated_frame, tuple(np.array(confidence_region[-1]) * np.array(annotated_frame.shape[:2][::-1])), 
        #          tuple(np.array(confidence_region[0]) * np.array(annotated_frame.shape[:2][::-1])), (0, 255, 0), 1)

        cv2.imshow("YOLOv8 Detection with bbox", annotated_frame)
        cv2.waitKey(1)
        
        # Publish detection results
        if len(results) > 0:
            result_list = (results[0].boxes.cls).cpu().tolist()
            xywhn_list = (results[0].boxes.xywhn).cpu().tolist()
        
        detection_info = [{'results': result_list, 'xywhn_list': xywhn_list}]
        
        self.detection_publisher.publish_detection_info(detection_info)

print('test2')
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

print('test3')
def main(args=None):
    print('test4')
    rclpy.init(args=args)
    print('test5')
    image_subscriber = BittlePlannerSubscriber()
    print('test6')
    rclpy.spin(image_subscriber)
    print('test7')
    image_subscriber.destroy_node()
    print('test8')
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
