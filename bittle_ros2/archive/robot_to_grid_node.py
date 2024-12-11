import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bittle_msgs.msg import Command, Detection
from sensor_msgs.msg import Imu
import threading
import time
import numpy as np

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(Command, 'command', 10)
        self.detection_subscription = self.create_subscription(
            Detection,
            'detection',
            self.detection_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        #self.imu_subscription = self.create_subscription(
        #    Imu,
        #    'imu',
        #    self.imu_callback,
        #    10,
        #    callback_group=ReentrantCallbackGroup()
        #)
        self.get_logger().info('Command Publisher Node has been started.')

        self.target_square = None
        self.current_heading = None
        self.current_position = None
        self.detections = []
        self.command_queue = []
        self.previous_robot_position = None
        self.crawl_threshold = 10
        self.turn_threshold = 40

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

    def detection_callback(self, msg):
        self.current_heading = msg.april_tag_orientation
        self.current_position = msg.april_tag_location
        centers = list(zip(*(iter(msg.center),) * 2))

        for class_name, grid_square, center in zip(msg.class_names, msg.grid_squares, centers):
            center_x, center_y = center[0], center[1]
            self.detections.append((class_name, grid_square, (center_x, center_y)))
        

        if self.target_square:
            self.navigate_to_target()


    def navigate_to_target(self):
        if self.current_heading is None:
            self.get_logger().info('Current heading not available.')
            return

        if not self.detections:
            self.get_logger().info('No detections available.')
            return

        # Get the position of the robot
        robot_position = self.current_position

        if not robot_position:
            self.get_logger().info('Robot not detected.')
            return

        self.previous_robot_position = robot_position

        target_position = self.grid_centers[self.target_square]
        direction_vector = (target_position[0] - robot_position[0], robot_position[1] - target_position[1])
        self.get_logger().info(f'Direction vector: {direction_vector}')
        self.move_towards_target(direction_vector)

    def move_towards_target(self, direction_vector):
        dx, dy = direction_vector
        self.get_logger().info(f'dx: {dx}, dy: {dy}')
        epsilon = 1e-6
        error = np.sqrt(dx**2 + dy**2) + epsilon
        self.get_logger().info(f'Error: {error}')
        Txhat = dx / error
        Tyhat = dy / error
        Rxhat = np.cos(self.current_heading * np.pi / 180)
        Ryhat = np.sin(self.current_heading * np.pi / 180)
        TdotR = Txhat * Rxhat + Tyhat * Ryhat
        TcrossR = Txhat * Ryhat - Tyhat * Rxhat
        magnitude_T = np.sqrt(Txhat**2 + Tyhat**2)
        magnitude_R = np.sqrt(Rxhat**2 + Ryhat**2)
        self.get_logger().info(f'TdotR: {TdotR}, magnitude_T: {magnitude_T}, magnitude_R: {magnitude_R}')
        self.get_logger().info(f'Txhat: {Txhat}, Tyhat: {Tyhat}, Rxhat: {Rxhat}, Ryhat: {Ryhat}')
        theta = np.arccos(np.clip(TdotR, -1.0, 1.0)) * 180 / np.pi
        if TcrossR < 0:
            theta = -theta
        self.get_logger().info(f'Theta: {theta}')
        self.get_logger().info(f'Current heading: {self.current_heading}')
        if error < 20:
            self.get_logger().info('Reached target square.')
            self.publish_command('krest',0.0)
        else:
            self.adjust_heading(theta)


    def adjust_heading(self, theta):
        if abs(theta) < self.crawl_threshold:
            self.publish_command('kcrF',0.0)
        elif abs(theta) > self.turn_threshold:
            if theta < 0:
                self.publish_command('kvtL',0.5)
            else:
                self.publish_command('kvtR',0.5)
        elif self.turn_threshold > abs(theta) > self.crawl_threshold:
            if theta < 0:
                self.publish_command('kcrL',0.5)
            else:
                self.publish_command('kcrR',0.5)


def main(args=None):
    rclpy.init(args=args)

    command_publisher = CommandPublisher()

    target_square = input("Enter target grid square (e.g., A1): ").strip().upper()
    command_publisher.target_square = target_square

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(command_publisher, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        command_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


