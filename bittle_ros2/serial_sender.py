import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import serial
from bittle_msgs.msg import Command
from bittle_msgs.msg import SerialResponse
import threading
import time

class SerialSender(Node):
    def __init__(self, port='/dev/ttyAMA0'):
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
            'serial_command_topic',
            self.command_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Publisher for serial responses
        self.response_publisher = self.create_publisher(
            SerialResponse,
            'serial_response_topic',
            10
        )
        
        self.last_command = None
        self.last_command_time = time.time()

        # Start a separate thread to read from serial port
        self.serial_thread = threading.Thread(target=self.read_from_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def command_callback(self, msg):
        current_time = time.time()
        if msg.cmd != self.last_command:
            self.wrapper([msg.cmd, 0])
            self.last_command = msg.cmd
            self.last_command_time = current_time

    def wrapper(self, task):
        if len(task) == 2:
            self.serialWriteByte([task[0]])

    def serialWriteByte(self, var=[]):
        instrStr = var[0] + '\n'
        self.get_logger().info(f"Sending: {instrStr.strip()}")
        self.ser.write(instrStr.encode())

    def read_from_serial(self):
        while True:
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()  # Read a line and decode it
                self.publish_response(response)

    def publish_response(self, response):
        msg = SerialResponse()
        msg.response = response
        self.response_publisher.publish(msg)
        self.get_logger().info(f"Published Serial Response: {response}")

def main(args=None):
    rclpy.init(args=args)
    serial_sender = SerialSender()
    
    executor = MultiThreadedExecutor()
    rclpy.spin(serial_sender, executor=executor)

    serial_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


