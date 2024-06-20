import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, 'imu_data', 1)  # Set buffer size to 1
        self.subscription = self.create_subscription(
            String,
            'command',
            self.command_callback,
            1)  # Set buffer size to 1
        self.subscription  # prevent unused variable warning

        self.port = '/dev/ttyAMA0'  # Ensure this is the correct port
        self.ser = serial.Serial(self.port, 115200, timeout=1)

        self.get_logger().info(f"Connected to {self.port}")

        self.latest_data = None
        self.data_lock = threading.Lock()

        self.velocity = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.0]
        self.last_time = time.time()

        # Start a thread to read IMU data
        self.read_thread = threading.Thread(target=self.read_data)
        self.read_thread.daemon = True
        self.read_thread.start()

        # Start a thread to publish IMU data
        self.publish_thread = threading.Thread(target=self.publish_data)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def read_data(self):
        while True:
            if self.ser.in_waiting > 0:
                raw_data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                formatted_data = self.format_imu_data(raw_data)
                if formatted_data:
                    with self.data_lock:
                        self.latest_data = formatted_data
            time.sleep(0.01)  # Small delay to avoid busy-waiting

    def publish_data(self):
        while rclpy.ok():
            if self.latest_data:
                with self.data_lock:
                    msg = String()
                    msg.data = self.latest_data
                    self.publisher_.publish(msg)
                    self.latest_data = None
            time.sleep(0.1)  # Adjust the sleep time as necessary

    def format_imu_data(self, data):
        try:
            # Split the data based on tab character
            values = data.split('\t')
            # Check if we have the correct number of values
            if len(values) == 7:
                yaw, pitch, roll, x_acc, y_acc, z_acc, world_acc = map(float, values)

                # Get the current time and calculate delta time
                current_time = time.time()
                dt = current_time - self.last_time
                self.last_time = current_time

                # Integrate acceleration to get velocity
                self.velocity[0] += (x_acc / 16384.0) * dt
                self.velocity[1] += (y_acc / 16384.0) * dt
                self.velocity[2] += (z_acc / 16384.0) * dt

                # Integrate velocity to get position
                self.position[0] += self.velocity[0] * dt
                self.position[1] += self.velocity[1] * dt
                self.position[2] += self.velocity[2] * dt

                # Create formatted string with labels
                formatted_data = (
                    f"Yaw: {yaw:.2f}, "
                    f"Velocity X: {self.velocity[0]:.2f}, "
                    f"Velocity Y: {self.velocity[1]:.2f}, "
                    f"Velocity Z: {self.velocity[2]:.2f}, "
                    f"Position X: {self.position[0]:.2f}, "
                    f"Position Y: {self.position[1]:.2f}, "
                    f"Position Z: {self.position[2]:.2f}"
                )
                return formatted_data
            else:
                return None
        except Exception as e:
            self.get_logger().error(f"Error formatting data: {e}")
            return None

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
