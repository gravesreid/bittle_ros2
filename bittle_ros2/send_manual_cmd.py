import rclpy
from rclpy.node import Node
from bittle_msgs.msg import Command

class SendManualCmd(Node):
    def __init__(self):
        super().__init__('send_manual_cmd')
        self.publisher = self.create_publisher(Command, 'command', 10)
        self.get_logger().info('send_manual_cmd node started')

    def send_cmd_from_terminal(self):
        while True:
            cmd = input('Enter command: ')
            msg = Command()
            msg.cmd.append(cmd)
            msg.delay.append(0)
            self.publisher.publish(msg)
            self.get_logger().info(f'Sending command: {cmd}')

def main(args=None):
    rclpy.init(args=args)
    send_manual_cmd = SendManualCmd()
    send_manual_cmd.send_cmd_from_terminal()
    rclpy.spin(send_manual_cmd)
    send_manual_cmd.destroy_node()
    rclpy.shutdown()