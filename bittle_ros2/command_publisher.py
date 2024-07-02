import rclpy
from rclpy.node import Node
from bittle_msgs.msg import Command

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(Command, 'command', 10)
        self.get_logger().info('Command Publisher Node has been started.')

    def publish_command(self, command, delay):
        msg = Command()
        msg.cmd = [command]
        msg.delay = [delay]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.cmd} with delay {msg.delay}')

def main(args=None):
    rclpy.init(args=args)

    command_publisher = CommandPublisher()

    try:
        while rclpy.ok():
            command = input("Enter command: ")
            if command.lower() == 'q':
                break
            delay = float(input("Enter delay: "))
            command_publisher.publish_command(command, delay)
    except KeyboardInterrupt:
        pass
    finally:
        command_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
