import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import RPi.GPIO as GPIO
import time
import math

# GPIO pin constants (BCM numbering)
# HAS TO BE ALL PWM pins
IN1 = 12  # Direction pin for left motor
IN2 = 13  # Direction pin for right motor
AN1 = 18  # PWM pin for left motor
AN2 = 19  # PWM pin for right motor

# Motor driver modes
class MODE:
    PWM_DIR = 0
    PWM_PWM = 1

# Cytron Motor Driver class
class CytronMD:
    def __init__(self, mode, pin1, pin2, frequency=1000):
        self._mode = mode
        self._pin1 = pin1
        self._pin2 = pin2
        self._frequency = frequency

        GPIO.setup(self._pin1, GPIO.OUT)
        GPIO.setup(self._pin2, GPIO.OUT)
        GPIO.output(self._pin1, GPIO.LOW)
        GPIO.output(self._pin2, GPIO.LOW)

        if self._mode == MODE.PWM_DIR:
            self.pwm1 = GPIO.PWM(self._pin1, self._frequency)
            self.pwm1.start(0)
            self.pwm2 = None
        elif self._mode == MODE.PWM_PWM:
            self.pwm1 = GPIO.PWM(self._pin1, self._frequency)
            self.pwm2 = GPIO.PWM(self._pin2, self._frequency)
            self.pwm1.start(0)
            self.pwm2.start(0)

    def setSpeed(self, speed):
        speed = max(min(speed, 255), -255)
        duty_cycle = (abs(speed) / 255.0) * 100

        if self._mode == MODE.PWM_DIR:
            self.pwm1.ChangeDutyCycle(duty_cycle)
            GPIO.output(self._pin2, GPIO.HIGH if speed < 0 else GPIO.LOW)
        elif self._mode == MODE.PWM_PWM:
            if speed >= 0:
                self.pwm1.ChangeDutyCycle(duty_cycle)
                self.pwm2.ChangeDutyCycle(0)
            else:
                self.pwm1.ChangeDutyCycle(0)
                self.pwm2.ChangeDutyCycle(duty_cycle)

    def cleanup(self):
        if self.pwm1:
            self.pwm1.stop()
        if self.pwm2:
            self.pwm2.stop()

# ROS Node
class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Initialize motor drivers (PWM_DIR mode)
        self.motor_left = CytronMD(MODE.PWM_DIR, AN1, IN1)  # Left: PWM=18, DIR=12
        self.motor_right = CytronMD(MODE.PWM_DIR, AN2, IN2)  # Right: PWM=19, DIR=13

        # Subscriptions
        self.throttle_sub = self.create_subscription(
            Float64, 'throttleSpeed', self.throttle_callback, 10)
        self.turn_sub = self.create_subscription(
            Float64, 'turnAngle', self.turn_callback, 10)

    def throttle_callback(self, msg):
        throttle = max(min(msg.data, 1.0), -1.0)  # Clamp to [-1, 1]
        self.get_logger().info(f'Throttle Speed [-1, 1]: {throttle}')
        speed = throttle * 255  # Map to [-255, 255]
        self.motor_left.setSpeed(int(speed))
        self.motor_right.setSpeed(int(speed))

    def turn_callback(self, msg):
        turn = max(min(msg.data, math.pi), -math.pi)  # Clamp to [-π, π]
        self.get_logger().info(f'Turn Angle [-PI, PI]: {turn}')
        pass

    def cleanup(self):
        self.motor_left.setSpeed(0)
        self.motor_right.setSpeed(0)
        self.motor_left.cleanup()
        self.motor_right.cleanup()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()

    rclpy.spin(motor_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()