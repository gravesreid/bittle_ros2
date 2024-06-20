import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, 'command', 10)
        self.get_logger().info('Command Publisher Node has been started.')

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    command_publisher = CommandPublisher()

    try:
        while rclpy.ok():
            command = input("Enter command: ")
            command += '\n'
            if command.lower() == 'q':
                break
            command_publisher.publish_command(command)
    except KeyboardInterrupt:
        pass
    finally:
        command_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
