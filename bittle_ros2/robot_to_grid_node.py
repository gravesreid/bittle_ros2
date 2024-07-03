import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bittle_msgs.msg import Command, Detection
from sensor_msgs.msg import Imu
import threading
import time

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(Command, 'command', 10)
        self.detection_subscription = self.create_subscription(
            Detection,
            'detection',
            self.detection_callback,
            1,
            callback_group=ReentrantCallbackGroup()
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            1,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info('Command Publisher Node has been started.')

        self.target_square = None
        self.current_heading = None
        self.detections = []
        self.command_queue = []

        self.magnetometer_map = {
            (-51, 10): 'north',
            (35, 10): 'east',
            (-145, 10): 'west',
            (120, 10): 'south'
        }

        self.grid_centers = {
            "A1": (80, 60), "A2": (240, 60), "A3": (400, 60), "A4": (560, 60),
            "B1": (80, 180), "B2": (240, 180), "B3": (400, 180), "B4": (560, 180),
            "C1": (80, 300), "C2": (240, 300), "C3": (400, 300), "C4": (560, 300),
            "D1": (80, 420), "D2": (240, 420), "D3": (400, 420), "D4": (560, 420)
        }

    def publish_command(self, command, delay):
        msg = Command()
        msg.cmd = [command]
        msg.delay = [delay]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.cmd} with delay {msg.delay}')