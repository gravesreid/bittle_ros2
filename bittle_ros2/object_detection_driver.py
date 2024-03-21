#!/usr/bin/env python3

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

dir_dict = {1: 'kwkF', -1: 'kbk', 2: 'kwkL', 3: 'kwkR', 0: 'kbalance', 4: 'kpone', 5: 'kthree', 6: 'kcollectF'}


class Driver(Node):

    def __init__(self, port='/dev/ttyAMA0'):
        super().__init__('cmd_vel_listener')
        self.dir = 0
        self.num_commands_sent = 0
        self.subscription = self.create_subscription(
            Detection,
            '/detection_topic',
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
        self.get_logger().info("Received a /detection_topic message!")

        results = msg.results
        xywhn_list = msg.xywhn_list

        print("Results: ", results)
        print("xywhn_list: ", xywhn_list)
        
        if len(results) > 0:
            if xywhn_list[0] > 0.75:
                dir = 0
            elif xywhn_list[0] < 0.25:
                dir = 0
            else:
                dir = 0
        elif self.num_commands_sent % 5 == 0:
            dir = 0
        else:
            dir = 0

        if self.dir != dir:
            self.wrapper([dir_dict[dir], 0])
            self.dir = dir
            self.num_commands_sent += 1

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

