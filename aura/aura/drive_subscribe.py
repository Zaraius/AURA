import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
from geometry_msgs.msg import Pose2D
import RPi.GPIO as GPIO
import time
import math
import threading
from simple_pid import PID
from aura.constants import MAX_SPEED, ENCODER_TICKS_PER_REV, WHEEL_RADIUS, WHEELBASE

# ============================================
# DRIVE GPIO PIN ASSIGNMENTS (BCM)
# ============================================
# Motor A (Left) - Uses PWM Channel 0
AN1 = 18   # PWM left motor (Hardware PWM0)
IN1 = 19    # DIR left motor (Digital)

# Motor B (Right) - Uses PWM Channel 1
AN2 = 20   # PWM right motor (Hardware PWM1)
IN2 = 26    # DIR right motor (Digital)

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
STEPS_PER_REV = 1600  # 1/2 microstepping (adjust controller accordingly)
STEP_SLEEP_TIME = 0.0005  # delay between step pin toggles
STEER_STEPPER_GEAR_RATIO = 8     # Gear ratio for steering

# ============================================
# PID CONTROL PARAMETERS
# ============================================
PID_UPDATE_RATE = 0.02           # PID update rate in seconds (50Hz)
PID_KP = 400.0                     # Proportional gain (tune this!)
PID_KI = 0.0                     # Integral gain (tune this!)
PID_KD = 2.0                     # Derivative gain (tune this!)

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

# Concurrent Stepper Motor Controller
class ConcurrentStepperMotor:
    def __init__(self, dir_pin, step_pin, name="stepper"):
        """
        Initialize stepper motor with concurrent control support.
        
        Args:
            dir_pin: GPIO pin for direction
            step_pin: GPIO pin for step signal
            name: Name for logging
        """
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.name = name
        
        # Position tracking
        self.current_angle = 0.0      # Current position in radians
        self.target_angle = 0.0       # Target position in radians
        self.current_steps = 0        # Current position in steps (for fine tracking)
        
        # Thread control
        self.thread = None
        self.running = False
        self.lock = threading.Lock()
        
        # Setup GPIO
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.output(self.dir_pin, CW)
        GPIO.output(self.step_pin, GPIO.LOW)
    
    def radians_to_steps(self, angle_radians):
        """Convert angle in radians to number of steps"""
        return int((angle_radians / (2 * math.pi)) * STEPS_PER_REV * STEER_STEPPER_GEAR_RATIO)
    
    def steps_to_radians(self, steps):
        """Convert steps to angle in radians"""
        return (steps / (STEPS_PER_REV * STEER_STEPPER_GEAR_RATIO)) * (2 * math.pi)
    
    def move_to_angle(self, target_angle):
        """
        Non-blocking command to move to target angle.
        Interrupts any current movement and starts new one.
        
        Args:
            target_angle: Target angle in radians (absolute position)
        """
        with self.lock:
            # Stop any current movement
            if self.running and self.thread is not None:
                self.running = False
                if self.thread.is_alive():
                    self.thread.join(timeout=0.5)
            
            # Set new target
            self.target_angle = target_angle
            
            # Start new movement thread
            self.running = True
            self.thread = threading.Thread(target=self._move_thread, daemon=True)
            self.thread.start()
    
    def _move_thread(self):
        """
        Thread function that performs the actual stepping.
        Can be interrupted by setting self.running = False.
        """
        # Calculate initial parameters
        target_steps = self.radians_to_steps(self.target_angle)
        steps_to_move = target_steps - self.current_steps
        
        if steps_to_move == 0:
            self.running = False
            return
        
        # Set direction
        direction = CW if steps_to_move > 0 else CCW
        GPIO.output(self.dir_pin, direction)
        
        # Calculate absolute steps to take
        abs_steps = abs(steps_to_move)
        step_direction = 1 if steps_to_move > 0 else -1
        
        # Perform stepping with interruption checks
        for i in range(abs_steps):
            if not self.running:
                # Movement interrupted - update position to where we actually are
                break
            
            # Step pulse
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(STEP_SLEEP_TIME)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(STEP_SLEEP_TIME)
            
            # Update current position
            self.current_steps += step_direction
            self.current_angle = self.steps_to_radians(self.current_steps)
        
        self.running = False
    
    def get_current_angle(self):
        """Thread-safe get current angle"""
        with self.lock:
            return self.current_angle
    
    def is_moving(self):
        """Check if stepper is currently moving"""
        return self.running
    
    def stop(self):
        """Stop the stepper motor immediately"""
        with self.lock:
            self.running = False
            if self.thread is not None and self.thread.is_alive():
                self.thread.join(timeout=0.5)
        
def calc_odometry(left_ticks, right_ticks, left_angle, right_angle, 
                  prev_left_ticks, prev_right_ticks,
                  current_x, current_y, current_theta,
                  wheel_radius, wheelbase_length, ticks_per_rev):
    """
    Calculate odometry for front-wheel drive Ackermann robot.
    NOTE: theta represents the direction the robot is trying to go (average front wheel direction).
    """
    
    distance_per_tick = (2 * math.pi * wheel_radius) / ticks_per_rev
    
    delta_left_ticks = left_ticks - prev_left_ticks
    delta_right_ticks = right_ticks - prev_right_ticks
    
    left_distance = delta_left_ticks * distance_per_tick
    right_distance = delta_right_ticks * distance_per_tick
    front_distance = (left_distance + right_distance) / 2.0
    
    # Theta is just the average of the front wheel angles (the direction wheels are pointing)
    avg_steering_angle = (left_angle + right_angle) / 2.0
    new_theta = avg_steering_angle
    new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))  # Normalize to [-pi, pi]
    
    if abs(avg_steering_angle) < 1e-6 or abs(front_distance) < 1e-6:
        # Straight wheels or no movement
        delta_x = front_distance * math.cos(new_theta)
        delta_y = front_distance * math.sin(new_theta)
    else:
        # Turning motion
        turn_radius = wheelbase_length / math.tan(avg_steering_angle)
        
        # Calculate arc length and position change based on wheel direction
        delta_x = front_distance * math.cos(new_theta)
        delta_y = front_distance * math.sin(new_theta)
    
    new_x = current_x + delta_x
    new_y = current_y + delta_y
    
    return new_x, new_y, new_theta

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

        # Initialize concurrent stepper motors
        self.stepper_left = ConcurrentStepperMotor(DIR_LEFT, STEP_LEFT, "left")
        self.stepper_right = ConcurrentStepperMotor(DIR_RIGHT, STEP_RIGHT, "right")

        # Subscriptions - commanded topic and encoder topics
        self.create_subscription(Float64MultiArray, '/commanded', self.drive_callback, 10)
        self.create_subscription(Int32, '/left_encoder', self.left_encoder_callback, 10)
        self.create_subscription(Int32, '/right_encoder', self.right_encoder_callback, 10)

        # Publisher for odometry
        self.odom_pub = self.create_publisher(Pose2D, '/odom', 10)

        # PID controllers - output is PWM (-255 to 255)
        self.pid_left = PID(PID_KP, PID_KI, PID_KD, setpoint=0)
        self.pid_right = PID(PID_KP, PID_KI, PID_KD, setpoint=0)
        # set software hard limit on the motor pwm output speed
        PWM_LIMIT = int(255)
        self.pid_left.output_limits = (-PWM_LIMIT, PWM_LIMIT)
        self.pid_right.output_limits = (-PWM_LIMIT, PWM_LIMIT)

        # Speed calculation variables
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.current_left_ticks = 0
        self.current_right_ticks = 0
        self.current_left_speed = 0.0   # m/s
        self.current_right_speed = 0.0  # m/s
        self.last_speed_update = time.time()

        # Target speeds from joystick
        self.target_left_speed = 0.0   # m/s
        self.target_right_speed = 0.0  # m/s

        # Odometry state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.odom_prev_left_ticks = 0
        self.odom_prev_right_ticks = 0

        # Create timer for PID control loop
        self.pid_timer = self.create_timer(PID_UPDATE_RATE, self.pid_control_loop)
        
        self.get_logger().info('Motor controller initialized with CONCURRENT control')
        self.get_logger().info(f'PID update rate: {PID_UPDATE_RATE}s ({1/PID_UPDATE_RATE:.1f}Hz)')
        self.get_logger().info(f'PID gains: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}')
        self.get_logger().info(f'Stepper control: NON-BLOCKING with interruption support')
        self.get_logger().info(f'Wheel radius: {WHEEL_RADIUS}m, Encoder TPR: {ENCODER_TICKS_PER_REV}')

    def left_encoder_callback(self, msg):
        """Update current left encoder tick count"""
        self.current_left_ticks = msg.data

    def right_encoder_callback(self, msg):
        """Update current right encoder tick count"""
        self.current_right_ticks = msg.data

    def calculate_speed(self):
        """
        Calculate current motor speeds in m/s from encoder changes.
        Should be called at a regular interval (PID_UPDATE_RATE).
        """
        current_time = time.time()
        dt = current_time - self.last_speed_update
        
        if dt <= 0:
            return  # Avoid division by zero
        
        # Calculate tick changes
        left_tick_delta = self.current_left_ticks - self.prev_left_ticks
        right_tick_delta = self.current_right_ticks - self.prev_right_ticks
        
        # Calculate wheel rotations
        left_rotations = left_tick_delta / ENCODER_TICKS_PER_REV
        right_rotations = right_tick_delta / ENCODER_TICKS_PER_REV
        
        # Calculate distance traveled (circumference * rotations)
        wheel_circumference = 2 * math.pi * WHEEL_RADIUS
        left_distance = left_rotations * wheel_circumference
        right_distance = right_rotations * wheel_circumference
        
        # Calculate speed (distance / time)
        self.current_left_speed = left_distance / dt
        self.current_right_speed = right_distance / dt
        
        # Update for next iteration
        self.prev_left_ticks = self.current_left_ticks
        self.prev_right_ticks = self.current_right_ticks
        self.last_speed_update = current_time

    def pid_control_loop(self):
        """
        Main PID control loop - runs at PID_UPDATE_RATE.
        Calculates current speed and applies PID control.
        """
        # Calculate current speeds from encoder changes
        self.calculate_speed()
        
        # Update PID setpoints (target speeds in m/s)
        self.pid_left.setpoint = self.target_left_speed
        self.pid_right.setpoint = self.target_right_speed
        
        # Calculate PID output (PWM values based on speed error)
        pwm_left = self.pid_left(self.current_left_speed)
        pwm_right = self.pid_right(self.current_right_speed)
        
        # Apply PWM to motors (DC motors are always responsive)
        self.motor_left.setSpeedPWM(pwm_left)
        self.motor_right.setSpeedPWM(pwm_right)
        
        # Get current stepper angles (thread-safe)
        left_angle = self.stepper_left.get_current_angle()
        right_angle = self.stepper_right.get_current_angle()
        
        # Update and publish odometry
        self.odom_x, self.odom_y, self.odom_theta = calc_odometry(
            self.current_left_ticks, self.current_right_ticks,
            left_angle, right_angle,
            self.odom_prev_left_ticks, self.odom_prev_right_ticks,
            self.odom_x, self.odom_y, self.odom_theta,
            WHEEL_RADIUS, WHEELBASE, ENCODER_TICKS_PER_REV
        )
        
        # Update previous ticks for odometry
        self.odom_prev_left_ticks = self.current_left_ticks
        self.odom_prev_right_ticks = self.current_right_ticks
        
        # Publish odometry
        odom_msg = Pose2D()
        odom_msg.x = self.odom_x
        odom_msg.y = self.odom_y
        odom_msg.theta = self.odom_theta
        self.odom_pub.publish(odom_msg)
        
        # Log every 10th update to avoid spam (every 0.2s at 50Hz)
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        self._log_counter += 1
        
        if self._log_counter >= 10:
            self._log_counter = 0
            # self.get_logger().info(
            #     f'Target: L={self.target_left_speed:.3f}m/s, R={self.target_right_speed:.3f}m/s | '
            #     f'Actual: L={self.current_left_speed:.3f}m/s, R={self.current_right_speed:.3f}m/s | '
            #     f'PWM: L={pwm_left:.1f}, R={pwm_right:.1f}'
            # )

    def drive_callback(self, msg):
        """
        Process motor commands from /commanded topic.
        Expected format: [target_left_mps, target_right_mps, fl_angle, fr_angle]
        Speeds are in m/s, angles are ABSOLUTE TARGET POSITIONS in radians.
        
        This is now completely non-blocking - all motors operate concurrently.
        """
        data = msg.data
        if len(data) != 4:
            self.get_logger().error('Expected 4 values in msg.data')
            return
        
        # Extract values
        self.target_left_speed = -float(data[0])   # m/s
        self.target_right_speed = -float(data[1])  # m/s
        fl_angle = float(data[2])                  # radians (absolute target)
        fr_angle = float(data[3])                  # radians (absolute target)
        
        self.get_logger().info(
            f'New command - Speeds: L={self.target_left_speed:.3f}m/s, '
            f'R={self.target_right_speed:.3f}m/s | '
            f'Angles: FL={fl_angle:.3f}rad, FR={fr_angle:.3f}rad'
        )
        
        # Command steppers to move to target angles (non-blocking, interrupts previous moves)
        self.stepper_left.move_to_angle(fl_angle)
        self.stepper_right.move_to_angle(fr_angle)
        
        # DC motor speeds are updated via target_left/right_speed and 
        # applied in the PID control loop which runs at 50Hz

    def cleanup(self):
        """Stop motors and cleanup GPIO."""
        self.get_logger().info('Cleaning up motors...')
        
        # Stop steppers
        self.stepper_left.stop()
        self.stepper_right.stop()
        
        # Stop DC motors
        self.motor_left.stop()
        self.motor_right.stop()
        
        # Cleanup GPIO
        # GPIO.cleanup()
        
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
