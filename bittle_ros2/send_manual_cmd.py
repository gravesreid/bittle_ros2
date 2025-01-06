import rclpy
from rclpy.node import Node
from bittle_msgs.msg import Command
from bittle_msgs.srv import SerialCommand

class SendManualCmd(Node):
    def __init__(self):
        super().__init__('send_manual_cmd')
        self.publisher = self.create_publisher(Command, 'command', 10)
        self.command_client = self.create_client(
            SerialCommand,
            'serial_command'
        )
        while not self.command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.command_request = SerialCommand.Request()
        self.get_logger().info('send_manual_cmd node started')

    def send_cmd_from_terminal(self):
        while True:
            cmd = input('Enter command: ')
            msg = Command()
            msg.cmd.append(cmd)
            msg.delay.append(0)
            self.publisher.publish(msg)
            self.get_logger().info(f'Sending command: {cmd}')
            # Command service
            self.command_request.cmd = cmd
            self.command_request.delay = 0.0
            self.future = self.command_client.call_async(self.command_request)
            rclpy.spin_until_future_complete(self, self.future)
            if self.future.result() is not None:
                self.get_logger().info(f'Received response: {self.future.result().reply}')
            else:
                self.get_logger().error('Service call failed')
            return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    send_manual_cmd = SendManualCmd()
    send_manual_cmd.send_cmd_from_terminal()
    rclpy.spin(send_manual_cmd)
    send_manual_cmd.destroy_node()
    rclpy.shutdown()