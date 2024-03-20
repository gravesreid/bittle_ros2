import rclpy
from rclpy.node import Node
import serial
import struct
import time

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
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
        
        #Initialize lists
        acorns = [] #Found Acorns
        white_pheromones = [] #Happy State
        black_pheromones = [] #Search State
        directions = [] #Stores direction motion commands
        x_distance = 0.5
        y_distance = 0.5

        # Loop through detected objects
        if len(results) > 0 and len(xywhn_list) > 0:
            for result, xywh in zip(results, [xywhn_list[i:i + 4] for i in range(0, len(xywhn_list), 4)]):
                # Extract x, y, w, h values for the current object
                x, y, w, h = xywh
                x_distance = 0.5 - x
                y_distance = 0.5 - y
                # Check the class of the current object and append it to the appropriate list
                if result == 1:  # Acorn
                    acorns.append((x, y, w, h))
                    # direction = 1  # Set direction to move towards the acorn
                elif result == 2:  # White Pheromone
                    white_pheromones.append((x, y, w, h))
                elif result == 0:  # Black Pheromone
                    black_pheromones.append((x, y, w, h))

        ################THIS IS THE CALCULATION PART################
                

        x_boundary_left = 0.5 - 0.1
        x_boundary_right = 0.5 + 0.1

        angle = np.arctan(y_distance/x_distance)


        ################THIS IS THE CALCULATION PART################

        if acorns:
            direction = self.rotate_to_item(x, angle, x_boundary_left, x_boundary_right)
            directions.append(direction)
            if self.dir != direction:
                self.wrapper([dir_dict[direction], 0])
                self.dir = direction
            direction = self.move_to_item(x, x_boundary_left, x_boundary_right)
            directions.append(direction)
            if self.dir != direction:
                self.wrapper([dir_dict[direction], 0])
                self.dir = direction
        
        elif white_pheromones and not acorns:
            direction = self.rotate_to_item(x, angle, x_boundary_left, x_boundary_right)
            directions.append(direction)
            if self.dir != direction:
                self.wrapper([dir_dict[direction], 0])
                self.dir = direction
            direction = self.move_to_item(x, x_boundary_left, x_boundary_right)
            if self.dir != direction:
                self.wrapper([dir_dict[direction], 0])
                self.dir = direction
            directions.append(direction)

        elif black_pheromones and not white_pheromones and not acorns:
            direction = self.rotate_to_item(x, angle, x_boundary_left, x_boundary_right)
            directions.append(direction)
            if self.dir != direction:
                self.wrapper([dir_dict[direction], 0])
                self.dir = direction
            direction = self.move_to_item(x, x_boundary_left, x_boundary_right)
            if self.dir != direction:
                self.wrapper([dir_dict[direction], 0])
                self.dir = direction
            directions.append(direction)
        
        elif not black_pheromones and not white_pheromones and not acorns:
            direction = self.rotate_bittle()
            if self.dir != direction:
                self.wrapper([dir_dict[direction], 0])
                self.dir = direction
            

        

    ##### USER DEFINED FUNCTIONS######    
    def rotate_to_item(self, x, angle, x_boundary_left, x_boundary_right):
        direction = 0
        if x < x_boundary_left or x > x_boundary_right:
            if angle > 0.1: #turn right
                direction = 3

            elif angle < -0.1: #turn left
                direction = 2 

        return direction
    
    def move_to_item(self, x, x_boundary_left, x_boundary_right):
        if x >= x_boundary_left or x <= x_boundary_right:
            direction = 1
        
        return direction

    # def move_to_pheromones(self, pheromone, msg):
        #assuming
        x_pheromone = pheromone[0]
        y_pheromone = pheromone[1]

        direction = 0
        #make narrow window
        x_distance = 0.5 - x_pheromone
        y_distance = 0.5 - y_pheromone

        self.x_boundary_left = 0.5 - 0.1
        self.x_boundary_right = 0.5 + 0.1

        angle = np.arctan(y_distance/x_distance) 

        if x_pheromone < self.x_boundary_left or x_pheromone > self.x_boundary_right:
            if angle > 0.1: #turn right
                direction = 3

            elif angle < -0.1: #turn left
                direction = 4 

        #RETURN STATEMENT HERE
                
        return direction
        
        #PROB ADD FUNCTION HERE FOR DIR=1
        #once the pheromone is within the narrow window, move forward towards it
        if x_pheromone >= self.x_boundary_left or x_pheromone <= self.x_boundary_right:
            direction = 1
            
    
       

    def rotate_bittle(self):
        direction = 2
        return direction



    ####ASK REID OR ABRAHAM ABOUT THIS#####    
    def queue_commands(self, direction, directions):
        directions.append(direction)



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


