import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import serial
from bittle_msgs.msg import Command
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
        
        # Subscription to the Command topic
        self.subscription = self.create_subscription(
            Command,
            'serial_command_topic',
            self.command_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        self.last_command = None
        self.last_command_time = time.time()

        # Timer to check for command sending interval
        self.timer = self.create_timer(0.1, self.timer_callback)

    def command_callback(self, msg):
        current_time = time.time()
        if msg.cmd != self.last_command:
            self.wrapper([msg.cmd, 0])
            self.last_command = msg.cmd
            self.last_command_time = current_time

    def timer_callback(self):
        current_time = time.time()
        if current_time - self.last_command_time >= 1:
            self.send_rest_command()

    def send_rest_command(self):
        if self.last_command != 'krest':
            self.wrapper(['krest', 0])
            self.last_command = 'krest'
            self.last_command_time = time.time()

    def wrapper(self, task):
        if len(task) == 2:
            self.serialWriteByte([task[0]])

    def serialWriteByte(self, var=[]):
        instrStr = var[0] + '\n'
        self.get_logger().info(f"Sending: {instrStr.strip()}")
        self.ser.write(instrStr.encode())

def main(args=None):
    rclpy.init(args=args)
    serial_sender = SerialSender()
    
    # Use MultiThreadedExecutor to allow timer and subscription callback to be processed concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(serial_sender, executor=executor)

    serial_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

