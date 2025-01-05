import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bittle_msgs.srv import SerialCommand
import threading
import time
from bittle_ros2.utils.SerialCommunication import Communication, port_list_number

class SerialSender(Node):
    def __init__(self, port='/dev/ttyS0'):
        super().__init__('serial_sender')
        self.communication = Communication(port, 115200, 1)

        self.response_server = self.create_service(
            SerialCommand,
            'serial_command',
            self.command_callback
        )
        
        self.serial_thread = threading.Thread(target=self.read_from_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def command_callback(self, request, response):
        cmd = request.cmd
        delay = request.delay
        self.send_command(cmd, response)
        time.sleep(delay)
        return response

    def send_command(self, cmd, response):
        instrStr = cmd + '\n'
        self.get_logger().info(f"Sending: {instrStr.strip()}")
        self.communication.Send_data(instrStr.encode('utf-8'))
        # read response
        response_from_UART = self.communication.Read_Line().decode('utf-8').strip()
        self.get_logger().info(f"Received: {response_from_UART}")
        if response_from_UART:
            response.reply = response_from_UART
        return response

    def read_from_serial(self):
        self.get_logger().info("Starting read_from_serial thread")
        while True:
            if self.communication.main_engine.in_waiting > 0:
                self.get_logger().info("Data available in serial buffer")
                response = self.communication.Read_Line().decode('utf-8').strip()
                self.get_logger().info(f"Received: {response}")
                if response:
                    if response == "k":
                        response = "received token"
                    else:
                        self.publish_response(response)
                else:
                    self.get_logger().info("Received empty response")
                self.communication.main_engine.reset_input_buffer()  # Clear the input buffer
            time.sleep(0.1)  # Add a small delay to avoid busy-waiting

def main(args=None):
    rclpy.init(args=args)
    Communication.Print_Used_Com()
    if not port_list_number:
        print("No serial ports found.")
        return
    serial_sender = SerialSender(port_list_number[0])
    print(f'Using port {port_list_number[0]}')
    
    executor = MultiThreadedExecutor()
    rclpy.spin(serial_sender, executor=executor)

    serial_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()