import rclpy
from rclpy.node import Node
import serial
import struct
import time
from bittle_msgs.srv import SerialCommand

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
        self.last_command = None  # Track the last command sent
        self.cli = self.create_client(SerialCommand, 'get_next_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.send_next_command()

    def send_next_command(self):
        req = SerialCommand.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))
        else:
            command_to_send = response.next_command
            if command_to_send == '':  # Assuming an empty string indicates no command
                if self.last_command != 'krest':
                    command_to_send = 'krest'
            self.wrapper([command_to_send, 0])
            self.last_command = command_to_send  # Update the last command sent

    def wrapper(self, task):  # Structure is [token, var=[], time]
        print(task)
        if len(task) == 2:
            self.serialWriteByte([task[0]])
        elif isinstance(task[1][0], int):
            self.serialWriteNumToByte(task[0], task[1])
        else:
            self.serialWriteByte(task[1])
        time.sleep(task[-1])

    def serialWriteNumToByte(self, token, var=[]):  # Only to be used for c m u b i l o within Python
        # print("Num Token "); print(token);print(" var ");print(var);print("\n\n");
        if token == 'l' or token == 'i':
            var = list(map(lambda x: int(x), var))
            instrStr = token + struct.pack('b' * len(var), *var) + '~'
        elif token == 'c' or token == 'm' or token == 'u' or token == 'b':
            instrStr = token + str(var[0]) + " " + str(var[1]) + '\n'
        print("!!!!" + instrStr)
        self.ser.write(instrStr.encode())

    def serialWriteByte(self, var=[]):
        token = var[0][0]
        if (token == 'c' or token == 'm' or token == 'b' or token == 'u') and len(var) >= 2:
            instrStr = ""
            for element in var:
                instrStr = instrStr + element + " "
        elif token == 'l' or token == 'i':
            if (len(var[0]) > 1):
                var.insert(1, var[0][1:])
            var[1:] = list(map(lambda x: int(x), var[1:]))
            instrStr = token + struct.pack('b' * len(var[1:]), *var[1:]) + '~'
        elif token == 'w' or token == 'k':
            instrStr = var[0] + '\n'
        else:
            instrStr = token
        print("!!!!!!! " + instrStr)
        self.ser.write(instrStr.encode())


def main(args=None):
    rclpy.init(args=args)
    serial_sender = SerialSender()
    rclpy.spin(serial_sender)

    serial_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()