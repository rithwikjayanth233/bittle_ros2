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
        direction = 0 

        results = msg.results
        xywhn_list = msg.xywhn_list
        # print("results")
        # print(xywhn_list)
        # print("-----")
        #Initialize lists
        acorns = [] #Found Acorns
        white_pheromones = [] #Happy State  
        black_pheromones = [] #Search State

        # Loop through detected objects
        if len(results) > 0 and len(xywhn_list) > 0:
            for result, xywh in zip(results, [xywhn_list[i:i + 4] for i in range(0, len(xywhn_list), 4)]):
                # Extract x, y, w, h values for the current object
                x, y, w, h = xywh

                # Check the class of the current object and append it to the appropriate list
                if result == 1:  # Acorn
                    acorns.append((x, y, w, h))
                    direction = 1  # Set direction to move towards the acorn
                elif result == 2:  # White Pheromone
                    white_pheromones.append((x, y, w, h))
                elif result == 0:  # Black Pheromone
                    black_pheromones.append((x, y, w, h))

        if self.dir != direction:
            self.wrapper([dir_dict[direction], 0])
            self.dir = direction
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


        
