import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control')
        self.servo_pin = 18  # Change to your GPIO pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)  # 50Hz frequency
        self.pwm.start(0)  # Initialization
        self.get_logger().info('Servo control node started')

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
    try:
        angle = 90  # Example angle
        servo_control_node.set_servo_angle(angle)
        rclpy.spin(servo_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        servo_control_node.pwm.stop()
        GPIO.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
