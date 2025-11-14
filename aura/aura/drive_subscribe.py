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
        self.drive_sub = self.create_subscription(
            Float64, 'commanded', self.drive_callback, 10)

    def drive_callback(self, msg):
        # Safely get the data field
        try:
            data = msg.data
        except Exception:
            self.get_logger().error('Received message without data field')
            return

        # Assume msg.data is a sequence of length 4: [throttle_left, throttle_right, fl_angle, fr_angle]
        if not hasattr(data, '__len__'):
            self.get_logger().error('Expected sequence of length 4 in msg.data')
            return
        try:
            throttle_left = max(min(float(data[0]), 1.0), -1.0)
            throttle_right = max(min(float(data[1]), 1.0), -1.0)
            fl_angle = float(data[2])
            fr_angle = float(data[3])
        except Exception as e:
            self.get_logger().error(f'Invalid msg.data format (expected 4 numeric values): {e}')
            return

        # Map [-1, 1] throttle to [-255, 255] motor command
        speed_left = int(throttle_left * 255)
        speed_right = int(throttle_right * 255)

        self.get_logger().info(f'Throttle L/R: {throttle_left:.3f}/{throttle_right:.3f} -> Speed L/R: {speed_left}/{speed_right}')
        # self.motor_left.setSpeed(speed_left)
        # self.motor_right.setSpeed(speed_right)

        # If you need to use angles elsewhere, they are available as fl_angle, fr_angle

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