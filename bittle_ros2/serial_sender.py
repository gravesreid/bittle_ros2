import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import serial
from bittle_msgs.msg import Command, SerialResponse
from sensor_msgs.msg import Imu
import threading
import time

class SerialSender(Node):
    def __init__(self, port='/dev/ttyAMA0'):
        super().__init__('serial_sender')
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        
        self.subscription = self.create_subscription(
            Command,
            'command',
            self.command_callback,
            10,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.response_publisher = self.create_publisher(
            SerialResponse,
            'imu_data_topic',
            10
        )
        
        self.imu_publisher = self.create_publisher(
            Imu,
            'imu',
            10
        )
        
        self.serial_thread = threading.Thread(target=self.read_from_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def command_callback(self, msg):
        for cmd, delay in zip(msg.cmd, msg.delay):
            self.send_command(cmd)
            time.sleep(delay)

    def send_command(self, cmd):
        instrStr = cmd + '\n'
        self.get_logger().info(f"Sending: {instrStr.strip()}")
        self.ser.write(instrStr.encode())

    def read_from_serial(self):
        while True:
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()
                self.ser.flushInput()  # Clear the input buffer
                self.publish_response(response)
                self.publish_imu(response)

    def publish_response(self, response):
        msg = SerialResponse()
        msg.response = response
        self.response_publisher.publish(msg)
        self.get_logger().info(f"Published IMU data: {response}")

    def publish_imu(self, response):
        try:
            parts = list(map(float, response.split('\t')))
            if len(parts) < 6:
                self.get_logger().error("Received incomplete IMU data")
                return
            
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Assuming yaw, pitch, roll are in degrees, and converting to radians
            imu_msg.orientation.x = parts[0]  # yaw
            imu_msg.orientation.y = parts[1]  # pitch
            imu_msg.orientation.z = parts[2]  # roll

            imu_msg.linear_acceleration.x = parts[3]
            imu_msg.linear_acceleration.y = parts[4]
            imu_msg.linear_acceleration.z = parts[5]

            self.imu_publisher.publish(imu_msg)
            self.get_logger().info(f"Published IMU sensor data: {response}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse IMU data: {e}")

def main(args=None):
    rclpy.init(args=args)
    serial_sender = SerialSender()
    
    executor = MultiThreadedExecutor()
    rclpy.spin(serial_sender, executor=executor)

    serial_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






