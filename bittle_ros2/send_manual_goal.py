import rclpy
from rclpy.node import Node
from bittle_msgs.msg import Point

class SendManualGoal(Node):
    def __init__(self):
        super().__init__('send_manual_goal')
        self.publisher = self.create_publisher(Point, 'goal_point', 10)
        self.get_logger().info('send_manual_goal node started')

    def send_goal_from_terminal(self):
        while True:
            x = int(input('Enter x: '))
            y = int(input('Enter y: '))
            angle = int(input('Enter angle: '))
            msg = Point()
            msg.x = x
            msg.y = y
            msg.angle = angle
            self.publisher.publish(msg)
            self.get_logger().info(f'Sending goal: x={x}, y={y}, angle={angle}')

def main(args=None):
    rclpy.init(args=args)
    send_manual_goal = SendManualGoal()
    send_manual_goal.send_goal_from_terminal()
    rclpy.spin(send_manual_goal)
    send_manual_goal.destroy_node()
    rclpy.shutdown()