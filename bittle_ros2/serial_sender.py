import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, 'imu_data', 10)
        self.subscription = self.create_subscription(
            String,
            'command',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.port = '/dev/ttyAMA0'  # Ensure this is the correct port
        self.ser = serial.Serial(self.port, 115200, timeout=1)

        self.get_logger().info(f"Connected to {self.port}")

        # Start a thread to read IMU data
        self.read_thread = threading.Thread(target=self.read_data)
        self.read_thread.daemon = True
        self.read_thread.start()

    def read_data(self):
        while True:
            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                msg = String()
                msg.data = data
                self.publisher_.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def command_callback(self, msg):
        command = msg.data
        self.send_command(command)

    def send_command(self, command):
        try:
            self.ser.write((command + '\n').encode())
            self.get_logger().info(f"Sent command: {command}")
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")

def main(args=None):
    rclpy.init(args=args)

    serial_node = SerialNode()

    rclpy.spin(serial_node)

    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
