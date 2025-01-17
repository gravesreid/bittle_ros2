import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import serial
from bittle_msgs.msg import Command, SerialResponse
from sensor_msgs.msg import Imu
import threading
import time


class SerialSender(Node):
    def __init__(self, port='/dev/ttyS0'):
        super().__init__('serial_sender')
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        
        self.subscription = self.create_subscription(
            Command,
            'command',
            self.command_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.response_publisher = self.create_publisher(
            SerialResponse,
            'response',
            10
        )


    def command_callback(self, msg):
        for cmd, delay in zip(msg.cmd, msg.delay):
            self.send_command(cmd)
            time.sleep(delay)

    def send_command(self, cmd):
        instrStr = cmd + '\n'
        self.get_logger().info(f"Sending: {instrStr.strip()}")
        self.ser.write(instrStr.encode())
        # read response
        response = self.ser.readline().decode().strip()
        self.get_logger().info(f"Received: {response}")




def main(args=None):
    rclpy.init(args=args)
    serial_sender = SerialSender()
    
    executor = MultiThreadedExecutor()
    rclpy.spin(serial_sender, executor=executor)

    serial_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






