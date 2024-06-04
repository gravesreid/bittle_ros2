#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct
import time
from bittle_msgs.srv import ExecuteCommand  # Import ExecuteCommand service

dir_dict = {'fwd': 'kwkF', 'back': 'kbk', 'left': 'kwkL', 'right': 'kwkR', 'rest': 'krest'}

class Driver(Node):

    def __init__(self, port='/dev/ttyAMA0'):
        super().__init__('driver_node')  # Update node name
        self.service = self.create_service(ExecuteCommand, 'execute_command', self.execute_command_callback)  # Create service server
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

    def execute_command_callback(self, request, response):
        command = request.command.strip('[] ').lower()  # Strip brackets and whitespace
        self.get_logger().info(f"Executing command: {command}")
        try:
            if command in dir_dict:
                # Execute the command
                self.wrapper([dir_dict[command], 3])
                # Set a timer to send the rest command after 3 seconds
                self.create_timer(3.0, self.send_rest_command)
                response.success = True
            else:
                self.get_logger().error(f"Unknown command: {command}")
                response.success = False
        except Exception as e:
            self.get_logger().error(f"Failed to execute command: {e}")
            response.success = False
        return response

    def send_rest_command(self):
        self.wrapper([dir_dict['rest'], 0])

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


