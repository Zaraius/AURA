import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import RPi.GPIO as GPIO

# ============================================
# CORRECTED GPIO PIN ASSIGNMENTS (BCM)
# ============================================
# Motor A (Left) - Uses PWM Channel 0
AN1 = 12   # PWM left motor (Hardware PWM0)
IN1 = 5    # DIR left motor (Digital)

# Motor B (Right) - Uses PWM Channel 1
AN2 = 13   # PWM right motor (Hardware PWM1)
IN2 = 6    # DIR right motor (Digital)

# Motor driver modes
class MODE:
    PWM_DIR = 0
    PWM_PWM = 1

# Cytron Motor Driver class
class CytronMD:
    def __init__(self, mode, pin_pwm, pin_dir, frequency=20000):
        """
        Initialize Cytron motor driver in PWM/DIR mode.
        
        Args:
            mode: MODE.PWM_DIR or MODE.PWM_PWM
            pin_pwm: BCM pin number for PWM (speed) - connects to AN pin
            pin_dir: BCM pin number for direction - connects to IN pin
            frequency: PWM frequency in Hz (default 20kHz)
        """
        self._mode = mode
        self._pin_pwm = pin_pwm  # AN pin (PWM)
        self._pin_dir = pin_dir  # IN pin (Digital)
        self._frequency = frequency

        # Setup GPIO pins
        GPIO.setup(self._pin_pwm, GPIO.OUT)
        GPIO.setup(self._pin_dir, GPIO.OUT)
        GPIO.output(self._pin_pwm, GPIO.LOW)
        GPIO.output(self._pin_dir, GPIO.LOW)

        # Initialize PWM only on the PWM pin (AN)
        if self._mode == MODE.PWM_DIR:
            self.pwm = GPIO.PWM(self._pin_pwm, self._frequency)
            self.pwm.start(0)
        elif self._mode == MODE.PWM_PWM:
            # Not used for MD30C in standard configuration
            self.pwm1 = GPIO.PWM(self._pin_pwm, self._frequency)
            self.pwm2 = GPIO.PWM(self._pin_dir, self._frequency)
            self.pwm1.start(0)
            self.pwm2.start(0)

    def setSpeed(self, speed):
        """
        Set motor speed and direction.
        
        Args:
            speed: -255 to +255 (negative = reverse, positive = forward)
        """
        # Clamp speed to valid range
        speed = max(min(speed, 255), -255)
        
        # Calculate duty cycle percentage
        duty_cycle = (abs(speed) / 255.0) * 100
        
        if self._mode == MODE.PWM_DIR:
            # Set PWM duty cycle on AN pin
            self.pwm.ChangeDutyCycle(duty_cycle)
            
            # Set direction on IN pin (Digital HIGH/LOW)
            if speed >= 0:
                GPIO.output(self._pin_dir, GPIO.LOW)   # Forward
            else:
                GPIO.output(self._pin_dir, GPIO.HIGH)  # Reverse
                
        elif self._mode == MODE.PWM_PWM:
            # Locked antiphase mode (not typically used)
            if speed >= 0:
                self.pwm1.ChangeDutyCycle(duty_cycle)
                self.pwm2.ChangeDutyCycle(0)
            else:
                self.pwm1.ChangeDutyCycle(0)
                self.pwm2.ChangeDutyCycle(duty_cycle)

    def cleanup(self):
        """Stop PWM and cleanup."""
        if hasattr(self, 'pwm'):
            self.pwm.stop()
        if hasattr(self, 'pwm1'):
            self.pwm1.stop()
        if hasattr(self, 'pwm2'):
            self.pwm2.stop()

# ROS Node
class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Initialize motor drivers in PWM_DIR mode
        # Left motor: PWM on AN1 (GPIO12), DIR on IN1 (GPIO5)
        self.motor_left = CytronMD(MODE.PWM_DIR, AN1, IN1)
        
        # Right motor: PWM on AN2 (GPIO13), DIR on IN2 (GPIO6)
        self.motor_right = CytronMD(MODE.PWM_DIR, AN2, IN2)

        # Subscribe to commanded topic
        self.drive_sub = self.create_subscription(
            Float64MultiArray, '/commanded', self.drive_callback, 10)
        
        self.get_logger().info('Motor controller initialized')
        self.get_logger().info('Left motor:  PWM=GPIO12(AN1), DIR=GPIO5(IN1)')
        self.get_logger().info('Right motor: PWM=GPIO13(AN2), DIR=GPIO6(IN2)')

    def drive_callback(self, msg):
        """
        Process motor commands from /commanded topic.
        Expected format: [throttle_left, throttle_right, fl_angle, fr_angle]
        """
        try:
            data = msg.data
        except Exception:
            self.get_logger().error('Received message without data field')
            return

        # Validate data length
        if not hasattr(data, '__len__') or len(data) != 4:
            self.get_logger().error('Expected 4 values in msg.data')
            return
        
        try:
            # Extract values
            throttle_left = float(data[0])   # -255 to +255
            throttle_right = float(data[1])  # -255 to +255
            fl_angle = round(float(data[2]), 2)
            fr_angle = round(float(data[3]), 2)
        except Exception as e:
            self.get_logger().error(f'Invalid data format: {e}')
            return

        # Log command
        self.get_logger().info(
            f'L={throttle_left:.1f}, R={throttle_right:.1f}, '
            f'FL={fl_angle:.2f}°, FR={fr_angle:.2f}°'
        )

        # Send to motors
        self.motor_left.setSpeed(throttle_left)
        self.motor_right.setSpeed(throttle_right)

    def cleanup(self):
        """Stop motors and cleanup GPIO."""
        self.get_logger().info('Cleaning up motors...')
        self.motor_left.setSpeed(0)
        self.motor_right.setSpeed(0)
        self.motor_left.cleanup()
        self.motor_right.cleanup()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.cleanup()
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
