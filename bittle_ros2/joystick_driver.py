#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct
import sys
import time
import math
from std_msgs.msg import String
from sensor_msgs.msg import Joy  # Import Joy message type

dir_dict = {1: 'kwkF', -1: 'kbk', 2: 'kwkL', 3: 'kwkR', 0: 'krest', 4: 'ksit', 5: 'kbuttUp',  6: 'kvtL', 7: 'kvtR', 8: 'kpherL',9: 'kpherR', 10: 'kcollect'}

class Driver(Node):

    def __init__(self, port='/dev/ttyAMA0'):
        super().__init__('joy_listener')  # Update node name if desired
        self.dir = 0
        self.subscription = self.create_subscription(
            Joy,  # Change message type to Joy
            '/joy',  # Change to the joystick topic
            self.callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

    def callback(self, msg):
        self.get_logger().info("Received a /joy message!")
        
        axes = msg.axes
        buttons = msg.buttons
        
        if axes[1] == 1:
            dir = 1
        elif axes[1] == -1:
            dir = -1
        elif axes[0] == -1:
            dir = 3
        elif axes[0] == 1:
            dir = 2
        elif buttons[0] == 1:
            dir = 9
        elif buttons[1] == 1:
            dir = 4
        elif buttons[2] == 1:
            dir = 5
        elif buttons[3] == 1:
            dir = 8
        elif buttons[4] == 1:
            dir = 6
        elif buttons[5] == 1:
            dir = 7
        elif buttons[9] == 1:
            dir = 10
        else:
            dir = 0

        if self.dir != dir:
            self.wrapper([dir_dict[dir], 0])
            self.dir = dir

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

