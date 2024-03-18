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

#Dictionairy of Bittle motions
dir_dict = {1: 'kcrF', -1: 'kbk', 2: 'kcrL', 3: 'kcrR', 0: 'kbalance'} 
#add commands, rotation steps
#

class BittlePathPlanner(Node):
    def __init__(self, port='/dev/ttyAMA0'):

        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.dir = 0
        self.model = YOLO('/home/rithwik/bittle/autonomous-bittle/vision-based-command/best.pt')
        

        self.subscription_1 = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.image_callback, 
            1)
        self.subscription_1  # prevent unused variable warning

        self.subscription_2 = self.create_subscription(
            Detection,
            '/detection_topic',
            self.pheromone_callback,
            10)
        self.subscription_2  # prevent unused variable warning

        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        # Bittle parameters (adjust as needed)
        self.bittle_center_x = 320  # X coordinate of the bottom center of the Bittle in the camera frame
        self.bittle_center_y = 240  # Y coordinate of the bottom center of the Bittle in the camera frame

    def image_callback(self, data):
        '''
        diplay camera feed from bittle camera
        '''
        try:
            self.get_logger().info('Receiving video frame')
            self.cv2_img = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
            cv2.imshow("camera", self.cv2_img)
            cv2.waitKey(1)

            # Perform object detection
            results = self.model([self.cv2_img])  
            
            annotated_frame = results[0].plot()

            cv2.imshow("YOLOv8 Detection with bbox", annotated_frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(str(e))

        


    def pheromone_callback(self, msg):
        '''
        Process camera feed for bead detection and path planning
        Implement logic to detect green and purple beads
        Implement path planning algorithm based on bead detection
        '''

        if self.cv2_img is not None:
            
            #object detection using YOLO
            results = self.model(self.cv2_img)
            
            #Initialize lists
            acorns = [] #Found Acorns
            green_pheromones = [] #Happy State
            black_pheromones = [] #Search State

            #looping through detected objects
            if len(results) > 0:
                result_list = (results[0].boxes.cls).cpu().tolist()
                xywhn_list = (results[0].boxes.xywhn).cpu().tolist()
                for i in range(len(xywhn_list)):
                    x, y, w, h, _ = xywhn_list[i]
                    if result_list[i] == 0.0:  # Green Pheromone
                        green_pheromones.append((x, y, w, h))
                    elif result_list[i] == 1.0:  # Black Pheromone
                        black_pheromones.append((x, y, w, h))
                    elif result_list[i] == 2.0:  # Acorn
                        acorns.append((x, y, w, h))
            
            #Move towards detected pheromones
            if acorns:
                self.move_to_pheromones(acorns[0])
            elif green_pheromones:
                self.move_to_pheromones(green_pheromones[0])
            elif black_pheromones:
                self.move_to_pheromones(black_pheromones[0])
            else:
                self.rotate_bittle()
        
    def move_to_pheromones(self, pheromone, msg):
        #assuming
        x_pheromone = pheromone[0]
        y_pheromone = pheromone[1]

        #make narrow window
        x_distance = self.bittle_center_x - x_pheromone
        y_distance = self.bittle_center_y - y_pheromone

        self.x_boundary_left = self.bittle_center_x - 50
        self.x_boundary_right = self.bittle_center_x + 50

        angle = np.arctan(x_distance/y_distance)

        while x_pheromone < self.x_boundary_left or x_pheromone > self.x_boundary_right:
            while angle > 0.1: #turn right
                try:
                    self.dir = 3
                    
                except CvBridgeError as e:
                    self.get_logger().error(str(e))
            while angle < -0.1: #turn left
                try:
                    self.dir = 4 
                except CvBridgeError as e:
                    self.get_logger().error(str(e))
            self.dir = 0
        
        #once the pheromone is within the narrow window, move forward towards it
        while x_pheromone >= self.x_boundary_left or x_pheromone <= self.x_boundary_right:
            buttons = msg.buttons
            axes = msg.axes
            try:
                self.dir = 1
                # time.sleep(1)
                # axes[1] == 0
            
            except CvBridgeError as e:
                self.get_logger().error(str(e))
            
            self.dir = 0
        

    def rotate_bittle(self, msg):
        '''
        If bot doesnt find any sort of pheromone, rotate it by 10 deg until it finds a pheromone or acorn
        '''
        buttons = msg.buttons
        axes = msg.axes
        try:
            axes[0] = -1 
            time.sleep(1)
        except CvBridgeError as e:
            self.get_logger().error(str(e))
                

    def joy_callback(self, msg):
        '''
        Capture joystick inputs for additional functionality if needed
        '''
        pass

    def boundary_callback (self, msg):
        '''
        Process camera feed to detect the blue tape boundary
        Calculate angle between tape line and boundary line to position the bot perpendicular to the tape
        '''
        try:
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
                

        except CvBridgeError as e:
            self.get_logger().error(str(e))


    #obstacle
    # def obstacle_avoidance (self, msg):
    #     ''''
    #     what should the bittle do when it detects the boundary and pheromones 
    #     acorn > green > black > boundary <> nothing
    #     '''

    #     try:
    #         pass
            
        
    #     except CvBridgeError as e:
    #         self.get_logger().error(str(e))


        
            
#prioritize pheromone over boundary

def main(args=None):
    rclpy.init(args=args)
    bittle_path_planner = BittlePathPlanner()
    rclpy.spin(bittle_path_planner)
    bittle_path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
