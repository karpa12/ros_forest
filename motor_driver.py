import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # Піни керування
        self.in1, self.in2 = 17, 18  # правий мотор
        self.in3, self.in4 = 22, 23  # лівий мотор
        self.enA, self.enB = 24, 25  # PWM

        GPIO.setmode(GPIO.BCM)
        for pin in [self.in1, self.in2, self.in3, self.in4, self.enA, self.enB]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, False)

        self.pwm_right = GPIO.PWM(self.enA, 1000)  # 1 кГц
        self.pwm_left = GPIO.PWM(self.enB, 1000)
        self.pwm_right.start(0)
        self.pwm_left.start(0)

        self.subscriber = self.create_subscription(String, 'cmd_move', self.listener_callback, 10)

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Command: {command}")
        self.stop()

        if command == 'w':
            self.forward(100, 100)
        elif command == 's':
            self.backward(100, 100)
        elif command == 'a':
            self.turn_left()
        elif command == 'd':
            self.turn_right()
        elif command == 'x':
            self.stop()
        elif command in ['w+a', 'a+w']:
            self.forward(50, 100)  # left slower
        elif command in ['w+d', 'd+w']:
            self.forward(100, 50)  # right slower
        elif command in ['s+a', 'a+s']:
            self.backward(50, 100)
        elif command in ['s+d', 'd+s']:
            self.backward(100, 50)

    def forward(self, right_speed, left_speed):
        GPIO.output(self.in1, True)
        GPIO.output(self.in2, False)
        GPIO.output(self.in3, True)
        GPIO.output(self.in4, False)
        self.pwm_right.ChangeDutyCycle(right_speed)
        self.pwm_left.ChangeDutyCycle(left_speed)

    def backward(self, right_speed, left_speed):
        GPIO.output(self.in1, False)
        GPIO.output(self.in2, True)
        GPIO.output(self.in3, False)
        GPIO.output(self.in4, True)
        self.pwm_right.ChangeDutyCycle(right_speed)
        self.pwm_left.ChangeDutyCycle(left_speed)

    def turn_left(self):
        GPIO.output(self.in1, False)
        GPIO.output(self.in2, True)
        GPIO.output(self.in3, True)
        GPIO.output(self.in4, False)
        self.pwm_right.ChangeDutyCycle(100)
        self.pwm_left.ChangeDutyCycle(100)

    def turn_right(self):
        GPIO.output(self.in1, True)
        GPIO.output(self.in2, False)
        GPIO.output(self.in3, False)
        GPIO.output(self.in4, True)
        self.pwm_right.ChangeDutyCycle(100)
        self.pwm_left.ChangeDutyCycle(100)

    def stop(self):
        for pin in [self.in1, self.in2, self.in3, self.in4]:
            GPIO.output(pin, False)
        self.pwm_right.ChangeDutyCycle(0)
        self.pwm_left.ChangeDutyCycle(0)


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        GPIO.cleanup()
        rclpy.shutdown()
