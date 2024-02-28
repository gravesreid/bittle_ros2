import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import String
from sensor_msgs.msg import Joy

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.servo_pin = 18  # Change to your GPIO pin
        self.angle = 0
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)  # 50Hz frequency
        self.pwm.start(0)  # Initialization
        self.get_logger().info('Servo control node started')
        
    def joy_callback(self, msg):
        # Get the joystick input
        x_axis = msg.axes[0]
        # Convert joystick input to servo angle
        angle = 90 + x_axis * 45
        self.set_servo_angle(angle)
        
    def set_servo_angle(self, angle):
        duty_cycle = self.angle_to_duty_cycle(angle)
        self.pwm.ChangeDutyCycle(duty_cycle)
        self.get_logger().info(f'Servo angle set to {angle} degrees')

    def angle_to_duty_cycle(self, angle):
        # Convert angle to duty cycle
        return angle / 18 + 2

def main(args=None):
    rclpy.init(args=args)
    servo_control_node = ServoControlNode()
    rclpy.spin(servo_control_node)
    GPIO.cleanup()
    servo_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
