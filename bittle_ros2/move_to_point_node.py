import rclpy
from rclpy.node import Node
from bittle_msgs.msg import AprilTag, Command, Point
from bittle_msgs.srv import SerialCommand
import numpy as np

'''
This node subscribes to the apriltag topic to get the position of the robot, and the goal point topic to get the goal point. It determines the appropriate command to send to the robot to reach the goal.
'''

class MoveToPointNode(Node):
    def __init__(self):
        super().__init__('move_to_point_node')
        self.apriltag_subscription = self.create_subscription(
            AprilTag,
            '/apriltag_topic',
            self.apriltag_callback,
            10)
        self.goal_point_subscription = self.create_subscription(
            Point,
            '/goal_point',
            self.goal_point_callback,
            10)
        self.command_client = self.create_client(
            SerialCommand,
            'serial_command'
        )
        while not self.command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.command_request = SerialCommand.Request()
        self.apriltag_positions = []
        self.goal_point = None
        self.current_heading = None
        self.current_position = None
        self.current_cmd = None
        self.detections = []
        self.command_queue = []
        self.crawl_threshold = 10
        self.turn_threshold = 40
        self.get_logger().info('move_to_point_node node started')


    def apriltag_callback(self, msg):
        self.apriltag_positions.clear()
        if len(msg.center) == 0:
            self.get_logger().info('No AprilTag detected.')
            self.send_command('krest', 0.0)
            return
        else:
            x, y = msg.center
            orientation = msg.orientation[-1]
            #self.get_logger().info('Received AprilTag detection: x=%d, y=%d, orientation=%d' % (x, y, orientation))
            self.apriltag_positions.append((x, y, orientation))
            self.current_heading = orientation

    def goal_point_callback(self, msg):
        self.goal_point = (msg.x, msg.y, msg.angle)
        self.get_logger().info('Received goal point: x=%d, y=%d, angle=%d' % (msg.x, msg.y, msg.angle))
        self.move_towards_target()

    def move_towards_target(self):
        dx = self.goal_point[0] - self.apriltag_positions[0][0]
        dy = self.goal_point[1] - self.apriltag_positions[0][1]
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
            self.send_command('krest',0.0)
        else:
            self.adjust_heading(theta)

    def adjust_heading(self, theta):
        if abs(theta) < self.crawl_threshold:
            self.send_command('kcrF', 0.0)
        elif abs(theta) > self.turn_threshold:
            if theta > 0:
                self.send_command('kvtL', 0.5)
            else:
                self.send_command('kvtR', 0.5)
        elif self.turn_threshold > abs(theta) > self.crawl_threshold:
            if theta > 0:
                self.send_command('kcrL', 0.5)
            else:
                self.send_command('kcrR', 0.5)

    def send_command(self, command, delay):
        if command != self.current_cmd:
            self.current_cmd = command
            self.command_request.cmd = command
            self.command_request.delay = delay
            self.future = self.command_client.call_async(self.command_request)
            rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    move_to_point_node = MoveToPointNode()
    rclpy.spin(move_to_point_node)
    move_to_point_node.destroy_node()
    rclpy.shutdown()

