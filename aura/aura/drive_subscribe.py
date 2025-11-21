import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
import RPi.GPIO as GPIO
import time
import math
from simple_pid import PID
from aura.constants import MAX_SPEED, ENCODER_TICKS_PER_REV, WHEEL_RADIUS

# ============================================
# DRIVE GPIO PIN ASSIGNMENTS (BCM)
# ============================================
# Motor A (Left) - Uses PWM Channel 0
AN1 = 13   # PWM left motor (Hardware PWM0)
IN1 = 5    # DIR left motor (Digital)

# Motor B (Right) - Uses PWM Channel 1
AN2 = 12   # PWM right motor (Hardware PWM1)
IN2 = 6    # DIR right motor (Digital)

# ============================================
# STEERING GPIO PIN ASSIGNMENTS (BCM)
# ============================================
# REMEMBER TO PLUG STEPPER CONTROLLER INTO 3.3V!!!
# Stepper motor pins
# Left stepper (front left wheel)
DIR_LEFT = 7  
STEP_LEFT = 1 

# Right stepper (front right wheel)
DIR_RIGHT = 17
STEP_RIGHT = 27

# Stepper motor constants
CW = 1
CCW = 0
STEPS_PER_REV = 400  # 1/2 microstepping (adjust controller accordingly)
STEP_SLEEP_TIME = 0.0005  # delay between step pin toggles

# ============================================
# SMOOTHING PARAMETERS - ADJUST THESE!
# ============================================
STEPPER_UPDATE_RATE = 0.01      # Time between stepper updates (0.01 = 100Hz)
SMOOTHING_ALPHA = 0.5            # Filter strength (0.1 = smooth, 0.9 = responsive)
MIN_ANGLE_MOVEMENT = 0.01        # Minimum angle change to move (radians)
STEER_STEPPER_GEAR_RATIO = 8     # Gear ratio for steering

# Motor driver modes
class MODE:
    PWM_DIR = 0
    PWM_PWM = 1

class CytronMD:
    def __init__(self, mode, pin_pwm, pin_dir, frequency=100):
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
        
        # IMPORTANT: Initialize to safe state BEFORE starting PWM
        GPIO.output(self._pin_pwm, GPIO.LOW)
        GPIO.output(self._pin_dir, GPIO.LOW)

        # Initialize PWM only on the PWM pin (AN)
        if self._mode == MODE.PWM_DIR:
            self.pwm = GPIO.PWM(self._pin_pwm, self._frequency)
            self.pwm.start(0)  # Start at 0% duty cycle
        elif self._mode == MODE.PWM_PWM:
            # Not used for MD30C in standard configuration
            self.pwm1 = GPIO.PWM(self._pin_pwm, self._frequency)
            self.pwm2 = GPIO.PWM(self._pin_dir, self._frequency)
            self.pwm1.start(0)
            self.pwm2.start(0)

    def setSpeedPWM(self, pwm_value):
        """
        Directly set PWM (0-255) with direction.
        
        Args:
            pwm_value: PWM value from -255 to 255 (negative = reverse, positive = forward)
        """
        # Calculate duty cycle percentage (0-100)
        duty_cycle = (abs(pwm_value) / 255.0) * 100.0
        
        if self._mode == MODE.PWM_DIR:
            # Set direction FIRST, then speed (safer)
            GPIO.output(self._pin_dir, GPIO.LOW if pwm_value >= 0 else GPIO.HIGH)
            # Set PWM duty cycle on AN pin
            self.pwm.ChangeDutyCycle(duty_cycle)
                
        elif self._mode == MODE.PWM_PWM:
            # Locked antiphase mode (not typically used)
            if pwm_value >= 0:
                self.pwm1.ChangeDutyCycle(duty_cycle)
                self.pwm2.ChangeDutyCycle(0)
            else:
                self.pwm1.ChangeDutyCycle(0)
                self.pwm2.ChangeDutyCycle(duty_cycle)

    def stop(self):
        """Stop motor completely"""
        if self._mode == MODE.PWM_DIR:
            self.pwm.ChangeDutyCycle(0)
            GPIO.output(self._pin_pwm, GPIO.LOW)
            GPIO.output(self._pin_dir, GPIO.LOW)
        elif self._mode == MODE.PWM_PWM:
            self.pwm1.ChangeDutyCycle(0)
            self.pwm2.ChangeDutyCycle(0)

# Stepper Motor Controller class with smoothing
class StepperMotor:
    def __init__(self, dir_pin, step_pin, update_interval=STEPPER_UPDATE_RATE, 
                 alpha=SMOOTHING_ALPHA, min_movement=MIN_ANGLE_MOVEMENT):
        """
        Initialize stepper motor with smoothing.
        
        Args:
            dir_pin: GPIO pin for direction
            step_pin: GPIO pin for step signal
            update_interval: Time between updates in seconds (lower = faster updates)
            alpha: Smoothing factor 0-1 (lower = smoother, higher = more responsive)
            min_movement: Minimum angle change in radians to trigger movement
        """
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        
        # Position tracking
        self.current_angle = 0.0      # Actual current position
        self.target_angle = 0.0       # Raw target from joystick
        self.filtered_angle = 0.0     # Smoothed target
        
        # Smoothing parameters
        self.update_interval = update_interval
        self.alpha = alpha
        self.min_movement = min_movement
        self.last_update_time = time.time()
        
        # Setup GPIO
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.output(self.dir_pin, CW)
        GPIO.output(self.step_pin, GPIO.LOW)
    
    def radians_to_steps(self, angle_radians):
        """Convert angle in radians to number of steps"""
        return int((angle_radians / (2 * math.pi)) * STEPS_PER_REV * STEER_STEPPER_GEAR_RATIO)
    
    def set_target_angle(self, angle):
        """
        Set new target angle from joystick input.
        This applies exponential moving average filtering.
        
        Args:
            angle: Target angle in radians
        """
        # Apply exponential moving average filter
        self.filtered_angle = self.alpha * angle + (1 - self.alpha) * self.filtered_angle
        self.target_angle = self.filtered_angle
    
    def update(self):
        """
        Periodically move stepper toward target angle.
        Call this regularly (e.g., from a timer callback).
        Returns True if motor moved, False otherwise.
        """
        # Check if enough time has passed since last update
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return False
        
        self.last_update_time = current_time
        
        # Calculate angle difference
        angle_diff = self.target_angle - self.current_angle
        
        # Don't move if difference is too small
        if abs(angle_diff) < self.min_movement:
            return False
        
        # Determine direction
        direction = CW if angle_diff >= 0 else CCW
        GPIO.output(self.dir_pin, direction)
        
        # Calculate steps needed
        steps = abs(self.radians_to_steps(angle_diff))
        
        # Move the motor
        for _ in range(steps):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(STEP_SLEEP_TIME)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(STEP_SLEEP_TIME)
        
        # Update current position
        self.current_angle = self.target_angle
        return True

# ROS Node
class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Initialize DC motor drivers in PWM_DIR mode
        self.motor_left = CytronMD(MODE.PWM_DIR, AN1, IN1)
        self.motor_right = CytronMD(MODE.PWM_DIR, AN2, IN2)

        # Initialize stepper motors for steering with smoothing
        self.stepper_left = StepperMotor(DIR_LEFT, STEP_LEFT)
        self.stepper_right = StepperMotor(DIR_RIGHT, STEP_RIGHT)

        # Subscriptions - commanded topic and encoder topics
        self.create_subscription(Float64MultiArray, '/commanded', self.drive_callback, 10)
        self.create_subscription(Int32, '/left_encoder', self.left_encoder_callback, 10)
        self.create_subscription(Int32, '/right_encoder', self.right_encoder_callback, 10)

        # PID controllers (P only for now)
        self.pid_left = PID(1.0, 0, 0, setpoint=0)
        self.pid_right = PID(1.0, 0, 0, setpoint=0)
        self.pid_left.output_limits = (-255, 255)
        self.pid_right.output_limits = (-255, 255)

        # Current encoder tick counts
        self.current_left_ticks = 0
        self.current_right_ticks = 0

        # Create timer to periodically update stepper positions
        timer_period = STEPPER_UPDATE_RATE / 2  # Check twice as often as update rate
        self.timer = self.create_timer(timer_period, self.update_steppers)
        
        self.get_logger().info('Motor controller initialized with PID and smoothing')
        self.get_logger().info(f'Stepper update rate: {STEPPER_UPDATE_RATE}s ({1/STEPPER_UPDATE_RATE:.1f}Hz)')
        self.get_logger().info(f'Smoothing alpha: {SMOOTHING_ALPHA}')
        self.get_logger().info(f'Min movement: {MIN_ANGLE_MOVEMENT} rad ({math.degrees(MIN_ANGLE_MOVEMENT):.2f} deg)')
        self.get_logger().info(f'Steps per rev: {STEPS_PER_REV}, Gear ratio: {STEER_STEPPER_GEAR_RATIO}')
        self.get_logger().info('Left motor:  PWM=GPIO12(AN2), DIR=GPIO5(IN2)')
        self.get_logger().info('Right motor: PWM=GPIO13(AN1), DIR=GPIO6(IN1)')

    def left_encoder_callback(self, msg):
        """Update current left encoder tick count"""
        self.current_left_ticks = msg.data

    def right_encoder_callback(self, msg):
        """Update current right encoder tick count"""
        self.current_right_ticks = msg.data

    def drive_callback(self, msg):
        """
        Process motor commands from /commanded topic.
        Expected format: [target_left_mps, target_right_mps, fl_angle, fr_angle]
        Speeds are in m/s, angles are in radians.
        """
        data = msg.data
        if len(data) != 4:
            self.get_logger().error('Expected 4 values in msg.data')
            return
        
        # Extract values
        target_left_mps = float(data[0])   # m/s
        target_right_mps = float(data[1])  # m/s
        fl_angle = float(data[2])          # radians
        fr_angle = float(data[3])          # radians

        # Convert target m/s to ticks per second
        # Wheel circumference = 2 * pi * radius = 2 * pi * WHEEL_RADIUS
        wheel_circumference = 2 * math.pi * WHEEL_RADIUS
        target_left_ticks = target_left_mps * ENCODER_TICKS_PER_REV / wheel_circumference
        target_right_ticks = target_right_mps * ENCODER_TICKS_PER_REV / wheel_circumference

        # Update PID setpoints
        self.pid_left.setpoint = target_left_ticks
        self.pid_right.setpoint = target_right_ticks

        # Calculate PID output (PWM values)
        pwm_left = self.pid_left(self.current_left_ticks)
        pwm_right = self.pid_right(self.current_right_ticks)

        self.get_logger().info(
            f'Target L={target_left_mps:.2f}m/s ({target_left_ticks:.1f}ticks/s), '
            f'R={target_right_mps:.2f}m/s ({target_right_ticks:.1f}ticks/s) | '
            f'Current L={self.current_left_ticks}ticks, R={self.current_right_ticks}ticks | '
            f'PWM L={pwm_left:.1f}, R={pwm_right:.1f} | '
            f'Angles FL={math.degrees(fl_angle):.2f}°, FR={math.degrees(fr_angle):.2f}°'
        )

        # Set motor speeds using PID output
        self.motor_left.setSpeedPWM(pwm_left)
        self.motor_right.setSpeedPWM(pwm_right)
        
        # Set target angles for stepper motors
        # self.stepper_left.set_target_angle(fl_angle)
        # self.stepper_right.set_target_angle(fr_angle)

    def update_steppers(self):
        """
        Timer callback to periodically update stepper motor positions.
        """
        self.stepper_left.update()
        self.stepper_right.update()

    def cleanup(self):
        """Stop motors and cleanup GPIO."""
        self.get_logger().info('Cleaning up motors...')
        
        # Stop DC motors
        self.motor_left.stop()
        self.motor_right.stop()
        
        # Cleanup GPIO
        GPIO.cleanup()
        
        self.get_logger().info('Cleanup complete')

def main(args=None):
    rclpy.init(args=args)
    node = DriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
