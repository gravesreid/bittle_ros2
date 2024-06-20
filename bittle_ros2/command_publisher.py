import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, 'command', 10)
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Command Publisher Node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = 'your_command_here'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    command_publisher = CommandPublisher()

    rclpy.spin(command_publisher)

    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
