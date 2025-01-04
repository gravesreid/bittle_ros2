import rclpy
from rclpy.node import Node
from bittle_msgs.msg import AprilTag, Command, Point
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
        self.command_publisher = self.create_publisher(
            Command,
            'command',
            10
        )
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
            self.publish_command('krest', 0.0)
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
            self.publish_command('krest',0.0)
        else:
            self.adjust_heading(theta)

    def adjust_heading(self, theta):
        if abs(theta) < self.crawl_threshold:
            self.publish_command('kcrF', 0.0)
        elif abs(theta) > self.turn_threshold:
            if theta > 0:
                self.publish_command('kvtL', 0.5)
            else:
                self.publish_command('kvtR', 0.5)
        elif self.turn_threshold > abs(theta) > self.crawl_threshold:
            if theta > 0:
                self.publish_command('kcrL', 0.5)
            else:
                self.publish_command('kcrR', 0.5)

    def publish_command(self, command, delay):
        if command != self.current_cmd:
            self.current_cmd = command
            msg = Command()
            msg.cmd = [command]
            msg.delay = [delay]
            self.command_publisher.publish(msg)
            self.get_logger().info(f'Publishing: {msg.cmd} with delay {msg.delay}')
        else:
            self.get_logger().info(f'Command already published: {command}')

def main(args=None):
    rclpy.init(args=args)
    move_to_point_node = MoveToPointNode()
    rclpy.spin(move_to_point_node)
    move_to_point_node.destroy_node()
    rclpy.shutdown()

