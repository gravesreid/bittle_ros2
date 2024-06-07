import rclpy
from rclpy.node import Node
import serial
import struct
from bittle_msgs.srv import ExecuteCommand

dir_dict = {'fwd': 'kwkF', 'back': 'kbk', 'left': 'kwkL', 'right': 'kwkR', 'rest': 'kbalance', 'spinleft': 'kvtL', 'spinright': 'kvtR'}

class Driver(Node):

    def __init__(self, port='/dev/ttyS0'):
        super().__init__('driver_node')
        self.service = self.create_service(ExecuteCommand, 'execute_command', self.execute_command_callback)
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        self.get_logger().info('Driver node initialized')

    def execute_command_callback(self, request, response):
        command = request.command.strip('[] ').lower()
        self.get_logger().info(f"Executing command: {command}")
        command_sent = False
        if command_sent is False:
            try:
                if command in dir_dict:
                    self.wrapper([dir_dict[command], 3])
                    self.create_timer(300.0, self.send_rest_command)
                    response.success = True
                    command_sent = True
                else:
                    self.get_logger().error(f"Unknown command: {command}")
                    response.success = False
            except Exception as e:
                self.get_logger().error(f"Failed to execute command: {e}")
                response.success = False
        return response

    def send_rest_command(self):
        self.get_logger().info("Sending rest command")
        self.wrapper([dir_dict['rest'], 0])

    def wrapper(self, task):
        self.get_logger().info(f"Wrapper task: {task}")
        if len(task) == 2:
            self.serialWriteByte([task[0]])
        elif isinstance(task[1][0], int):
            self.serialWriteNumToByte(task[0], task[1])
        else:
            self.serialWriteByte(task[1])
        self.get_logger().info(f"Sleeping for {task[-1]} seconds")
        self.create_timer(task[-1], self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Timer completed")

    def serialWriteNumToByte(self, token, var=[]):
        self.get_logger().info(f"serialWriteNumToByte token: {token}, var: {var}")
        if token in ['l', 'i']:
            var = list(map(int, var))
            instrStr = token + struct.pack('b' * len(var), *var) + '~'
        elif token in ['c', 'm', 'u', 'b']:
            instrStr = token + str(var[0]) + " " + str(var[1]) + '\n'
        self.get_logger().info(f"Sending instruction string: {instrStr}")
        self.ser.write(instrStr.encode())

    def serialWriteByte(self, var=[]):
        token = var[0][0]
        self.get_logger().info(f"serialWriteByte token: {token}, var: {var}")
        if token in ['c', 'm', 'b', 'u'] and len(var) >= 2:
            instrStr = " ".join(var) + " "
        elif token in ['l', 'i']:
            if len(var[0]) > 1:
                var.insert(1, var[0][1:])
            var[1:] = list(map(int, var[1:]))
            instrStr = token + struct.pack('b' * len(var[1:]), *var[1:]) + '~'
        elif token in ['w', 'k']:
            instrStr = var[0] + '\n'
        else:
            instrStr = token
        self.get_logger().info(f"Sending instruction string: {instrStr}")
        self.ser.write(instrStr.encode())

def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




