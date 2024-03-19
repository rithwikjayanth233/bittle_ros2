# import rclpy
# from rclpy.node import Node
# import serial
# import struct
# import time

# from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import os
# from ultralytics import YOLO
# import numpy as np
# import serial
# from bittle_msgs.msg import Detection

# dir_dict = {1: 'kcrF', -1: 'kbk', 2: 'kcrL', 3: 'kcrR', 0: 'kbalance'}

# class Driver(Node):

#     def __init__(self, port='/dev/ttyAMA0'):
#         super().__init__('cmd_vel_listener')
#         self.dir = 0
#         self.subscription = self.create_subscription(
#             Detection,
#             '/detection_topic',
#             self.callback,
#             1)
#         self.subscription  # prevent unused variable warning
#         self.ser = serial.Serial(
#             port=port,
#             baudrate=115200,
#             parity=serial.PARITY_NONE,
#             stopbits=serial.STOPBITS_ONE,
#             bytesize=serial.EIGHTBITS,
#             timeout=1
#         )

#     def callback(self, msg: Detection):
#         self.get_logger().info("Received a /detection_topic message!")

#         results = msg.results
#         xywhn_list = msg.xywhn_list
        
#         #Initialize lists
#         acorns = [] #Found Acorns
#         white_pheromones = [] #Happy State
#         black_pheromones = [] #Search State

#         #looping through detected objects
#         if len(results) > 0:
#             result_list = (results[0].boxes.cls).cpu().tolist()
#             xywhn_list = (results[0].boxes.xywhn).cpu().tolist()
#             for i in range(len(xywhn_list)):
#                 x, y, w, h, _ = xywhn_list[i]
#                 if result_list[i] == 0.0:  # white Pheromone
#                     white_pheromones.append((x, y, w, h))
#                 elif result_list[i] == 1.0:  # Black Pheromone
#                     black_pheromones.append((x, y, w, h))
#                 elif result_list[i] == 2.0:  # Acorn
#                     acorns.append((x, y, w, h))
        
#         #Move towards detected pheromones
#         if acorns:
#             self.move_to_pheromones(acorns[0])
#         elif white_pheromones:
#             self.move_to_pheromones(white_pheromones[0])
#         elif black_pheromones:
#             self.move_to_pheromones(black_pheromones[0])
#         else:
#             self.rotate_bittle()
        
#     def pheromone_callback(self, msg):
#         '''
#         Process camera feed for bead detection and path planning
#         Implement logic to detect white and purple beads
#         Implement path planning algorithm based on bead detection
#         '''

#         if self.cv2_img is not None:
            
#             #object detection using YOLO
#             results = self.model(self.cv2_img)
            
#             #Initialize lists
#             acorns = [] #Found Acorns
#             white_pheromones = [] #Happy State
#             black_pheromones = [] #Search State

#             #looping through detected objects
#             if len(results) > 0:
#                 result_list = (results[0].boxes.cls).cpu().tolist()
#                 xywhn_list = (results[0].boxes.xywhn).cpu().tolist()
#                 for i in range(len(xywhn_list)):
#                     x, y, w, h, _ = xywhn_list[i]
#                     if result_list[i] == 2.0:  # White Pheromone
#                         white_pheromones.append((x, y, w, h))
#                     elif result_list[i] == 1.0:  # Black Pheromone
#                         black_pheromones.append((x, y, w, h))
#                     elif result_list[i] == 0.0:  # Acorn
#                         acorns.append((x, y, w, h))
            
#             #Move towards detected pheromones
#             if acorns:
#                 self.move_to_pheromones(acorns[0])
#             elif white_pheromones and not acorns:
#                 self.move_to_pheromones(white_pheromones[0])
#             elif black_pheromones and not white_pheromones and not acorns:
#                 self.move_to_pheromones(black_pheromones[0])
#             elif not black_pheromones and not white_pheromones and not acorns:
#                 self.rotate_bittle()
        
#     def move_to_pheromones(self, pheromone, msg):
#         #assuming
#         x_pheromone = pheromone[0]
#         y_pheromone = pheromone[1]

#         #make narrow window
#         x_distance = 0.5 - x_pheromone
#         y_distance = 0.5 - y_pheromone

#         self.x_boundary_left = 0.5 - 0.1
#         self.x_boundary_right = 0.5 + 0.1

#         angle = np.arctan(y_distance/x_distance) 

#         while x_pheromone < self.x_boundary_left or x_pheromone > self.x_boundary_right:
#             while angle > 0.1: #turn right
#                 try:
#                     self.dir = 3
                    
#                 except CvBridgeError as e:
#                     self.get_logger().error(str(e))
                    
#             while angle < -0.1: #turn left
#                 try:
#                     self.dir = 4 
#                 except CvBridgeError as e:
#                     self.get_logger().error(str(e))
                    
#             self.dir = 0
        
#         #once the pheromone is within the narrow window, move forward towards it
#         while x_pheromone >= self.x_boundary_left or x_pheromone <= self.x_boundary_right:
#             try:
#                 self.dir = 1
#                 # time.sleep(1)
#                 # axes[1] == 0
            
#             except CvBridgeError as e:
#                 self.get_logger().error(str(e))
            
#         self.dir = 0
        

#     def rotate_bittle(self, msg):
#         '''
#         If bot doesnt find any sort of pheromone, rotate it by 10 deg until it finds a pheromone or acorn
#         '''
#         try:
#             self.dir = -1
#             time.sleep(1)
#         except CvBridgeError as e:
#             self.get_logger().error(str(e))

    


#     def wrapper(self, task):  # Structure is [token, var=[], time]
#         print(task)
#         if len(task) == 2:
#             self.serialWriteByte([task[0]])
#         elif isinstance(task[1][0], int):
#             self.serialWriteNumToByte(task[0], task[1])
#         else:
#             self.serialWriteByte(task[1])
#         time.sleep(task[-1])

#     def serialWriteNumToByte(self, token, var=[]):  # Only to be used for c m u b i l o within Python
#         # print("Num Token "); print(token);print(" var ");print(var);print("\n\n");
#         if token == 'l' or token == 'i':
#             var = list(map(lambda x: int(x), var))
#             instrStr = token + struct.pack('b' * len(var), *var) + '~'
#         elif token == 'c' or token == 'm' or token == 'u' or token == 'b':
#             instrStr = token + str(var[0]) + " " + str(var[1]) + '\n'
#         print("!!!!" + instrStr)
#         self.ser.write(instrStr.encode())

#     def serialWriteByte(self, var=[]):
#         token = var[0][0]
#         if (token == 'c' or token == 'm' or token == 'b' or token == 'u') and len(var) >= 2:
#             instrStr = ""
#             for element in var:
#                 instrStr = instrStr + element + " "
#         elif token == 'l' or token == 'i':
#             if (len(var[0]) > 1):
#                 var.insert(1, var[0][1:])
#             var[1:] = list(map(lambda x: int(x), var[1:]))
#             instrStr = token + struct.pack('b' * len(var[1:]), *var[1:]) + '~'
#         elif token == 'w' or token == 'k':
#             instrStr = var[0] + '\n'
#         else:
#             instrStr = token
#         print("!!!!!!! " + instrStr)
#         self.ser.write(instrStr.encode())


# def main(args=None):
#     rclpy.init(args=args)
#     driver = Driver()
#     rclpy.spin(driver)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     driver.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
import serial
import struct
import time
import numpy as np
import serial
from bittle_msgs.msg import Detection

dir_dict = {1: 'kcrF', -1: 'kbk', 2: 'kcrL', 3: 'kcrR', 0: 'kbalance'}

class Driver(Node):

    def __init__(self, port='/dev/ttyAMA0'):
        super().__init__('cmd_vel_listener')
        self.dir = 0
        self.subscription = self.create_subscription(
            Detection,
            '/detection_topic',
            self.callback,
            1)
        self.subscription  # prevent unused variable warning
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

    def callback(self, msg: Detection):
        self.get_logger().info("Received a /detection_topic message!")

        results = msg.results
        xywhn_list = msg.xywhn_list
        print("results")
        print(xywhn_list)
        print("-----")
        #Initialize lists
        acorns = [] #Found Acorns
        white_pheromones = [] #Happy State
        black_pheromones = [] #Search State

        #looping through detected objects
        if len(results) > 0:
        #     # result_list = (results[0].boxes.cls).cpu().tolist()
        #     # xywhn_list = (results[0].boxes.xywhn).cpu().tolist()
            for i in range(len(xywhn_list)):
                x, y, w, h = xywhn_list[i * 4:i * 4 + 4]


                if results[0] == 2:  # white Pheromone
                    white_pheromones.append((x, y, w, h))
                elif results[0] == 0:  # Black Pheromone
                    black_pheromones.append((x, y, w, h))
                elif results[0] == 1:  # Acorn
                    acorns.append((x, y, w, h))
                    self.dir=1

        
        # calculate what actions need to be taken

        #first lets have it rotate and move to the pheromone
                    
    
    def wrapper(self, task):  # Structure is [token, var=[], time]
        print(task)
        if len(task) == 2:
            self.serialWriteByte([task[0]])
        elif isinstance(task[1][0], int):
            self.serialWriteNumToByte(task[0], task[1])
        else:
            self.serialWriteByte(task[1])
        time.sleep(task[-1])

    def serialWriteNumToByte(self, token, var=[]):  # Only to be used for c m u b i l o within Python
        # print("Num Token "); print(token);print(" var ");print(var);print("\n\n");
        if token == 'l' or token == 'i':
            var = list(map(lambda x: int(x), var))
            instrStr = token + struct.pack('b' * len(var), *var) + '~'
        elif token == 'c' or token == 'm' or token == 'u' or token == 'b':
            instrStr = token + str(var[0]) + " " + str(var[1]) + '\n'
        print("!!!!" + instrStr)
        self.ser.write(instrStr.encode())

    def serialWriteByte(self, var=[]):
        token = var[0][0]
        if (token == 'c' or token == 'm' or token == 'b' or token == 'u') and len(var) >= 2:
            instrStr = ""
            for element in var:
                instrStr = instrStr + element + " "
        elif token == 'l' or token == 'i':
            if (len(var[0]) > 1):
                var.insert(1, var[0][1:])
            var[1:] = list(map(lambda x: int(x), var[1:]))
            instrStr = token + struct.pack('b' * len(var[1:]), *var[1:]) + '~'
        elif token == 'w' or token == 'k':
            instrStr = var[0] + '\n'
        else:
            instrStr = token
        print("!!!!!!! " + instrStr)
        self.ser.write(instrStr.encode())


def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


        

