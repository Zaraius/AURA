import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
import RPi.GPIO as GPIO
import time
import math
import threading
from simple_pid import PID
from aura.constants import MAX_SPEED, ENCODER_TICKS_PER_REV, WHEEL_RADIUS, WHEELBASE

# ============================================
# SAFETY CONSTANTS
# ============================================
MIN_SAFE_DISTANCE = 0.6  

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

# Concurrent DC Motor Controller with PID
class ConcurrentDCMotor:
    def __init__(self, motor_driver, encoder_ticks_per_rev, wheel_radius, kp, ki, kd, name="motor"):
        """
        Initialize DC motor with concurrent PID control.
        
        Args:
            motor_driver: CytronMD instance
            encoder_ticks_per_rev: Encoder ticks per revolution
            wheel_radius: Wheel radius in meters
            kp, ki, kd: PID gains
            name: Name for logging
        """
        self.motor = motor_driver
        self.name = name
        self.encoder_tpr = encoder_ticks_per_rev
        self.wheel_radius = wheel_radius
        
        # PID controller
        PWM_LIMIT = 255
        self.pid = PID(kp, ki, kd, setpoint=0)
        self.pid.output_limits = (-PWM_LIMIT, PWM_LIMIT)
        
        # State variables
        self.target_speed = 0.0  # m/s
        self.current_speed = 0.0  # m/s
        self.current_ticks = 0
        self.prev_ticks = 0
        self.last_update = time.time()
        
        # Thread control
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        
    def update_encoder(self, ticks):
        """Update encoder tick count (called from encoder callback)"""
        with self.lock:
            self.current_ticks = ticks
    
    def set_target_speed(self, speed_mps):
        """Set target speed in m/s"""
        with self.lock:
            self.target_speed = speed_mps
    
    def get_current_speed(self):
        """Get current speed in m/s"""
        with self.lock:
            return self.current_speed
    
    def start_control_loop(self, update_rate=0.02):
        """Start the PID control loop in a separate thread"""
        if self.running:
            return
        
        self.running = True
        self.update_rate = update_rate
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()
    
    def _control_loop(self):
        """PID control loop running in separate thread"""
        while self.running:
            loop_start = time.time()
            
            with self.lock:
                # Calculate speed
                current_time = time.time()
                dt = current_time - self.last_update
                
                if dt > 0:
                    tick_delta = self.current_ticks - self.prev_ticks
                    rotations = tick_delta / self.encoder_tpr
                    distance = rotations * (2 * math.pi * self.wheel_radius)
                    self.current_speed = distance / dt
                    
                    self.prev_ticks = self.current_ticks
                    self.last_update = current_time
                
                # Update PID
                self.pid.setpoint = self.target_speed
                pwm_output = self.pid(self.current_speed)
                
            # Apply PWM (outside lock to minimize lock time)
            self.motor.setSpeedPWM(pwm_output)
            
            # Sleep for remainder of update period
            elapsed = time.time() - loop_start
            sleep_time = max(0, self.update_rate - elapsed)
            time.sleep(sleep_time)
    
    def stop(self):
        """Stop the control loop and motor"""
        self.running = False
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.motor.stop()

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
                
def normalize_angle(angle):
    """Normalize angle to range [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def calc_odometry(left_ticks, right_ticks, left_angle, right_angle, 
                  prev_left_ticks, prev_right_ticks,
                  current_x, current_y, current_theta,
                  wheel_radius, wheelbase_length, ticks_per_rev):
    """
    Calculate odometry for rear-wheel drive Ackermann robot using Bicycle Model.
    
    Args:
        wheelbase_length: Distance between FRONT and REAR axles (meters)
    """
    
    # 1. Calculate distance traveled by each rear wheel
    distance_per_tick = (2 * math.pi * wheel_radius) / ticks_per_rev
    delta_left_ticks = left_ticks - prev_left_ticks
    delta_right_ticks = right_ticks - prev_right_ticks
    
    d_left = delta_left_ticks * distance_per_tick
    d_right = delta_right_ticks * distance_per_tick
    
    # Average distance traveled by the center of the rear axle
    dist_traveled = (d_left + d_right) / 2.0
    
    # 2. Calculate average steering angle (delta)
    # We use the average of the two front steppers to approximate the "virtual" center wheel
    steering_angle = (left_angle + right_angle) / 2.0
    
    # 3. Calculate change in heading (theta)
    # Kinematic equation: delta_theta = (distance / length) * tan(steering_angle)
    if abs(math.tan(steering_angle)) < 1e-6:
        # moving straight
        delta_theta = 0
    else:
        delta_theta = (dist_traveled / wheelbase_length) * math.tan(steering_angle)
    
    # 4. Calculate new pose
    # We use the "Runge-Kutta 2nd order" (midpoint) approximation for better accuracy 
    # by adding half the turn angle to the current heading for the position update.
    avg_theta = current_theta + (delta_theta / 2.0)
    
    delta_x = dist_traveled * math.cos(avg_theta)
    delta_y = dist_traveled * math.sin(avg_theta)
    
    # Update state
    new_x = current_x + delta_x
    new_y = current_y + delta_y
    new_theta = normalize_angle(current_theta + delta_theta)
    
    return new_x, new_y, new_theta

# ROS Node
class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Initialize DC motor drivers
        motor_driver_left = CytronMD(MODE.PWM_DIR, AN1, IN1)
        motor_driver_right = CytronMD(MODE.PWM_DIR, AN2, IN2)
        
        # Initialize concurrent DC motor controllers
        self.motor_left = ConcurrentDCMotor(
            motor_driver_left, ENCODER_TICKS_PER_REV, WHEEL_RADIUS,
            PID_KP, PID_KI, PID_KD, name="left"
        )
        self.motor_right = ConcurrentDCMotor(
            motor_driver_right, ENCODER_TICKS_PER_REV, WHEEL_RADIUS,
            PID_KP, PID_KI, PID_KD, name="right"
        )
        
        # Start DC motor control loops
        self.motor_left.start_control_loop(PID_UPDATE_RATE)
        self.motor_right.start_control_loop(PID_UPDATE_RATE)

        # Initialize concurrent stepper motors
        self.stepper_left = ConcurrentStepperMotor(DIR_LEFT, STEP_LEFT, "left")
        self.stepper_right = ConcurrentStepperMotor(DIR_RIGHT, STEP_RIGHT, "right")

        # Subscriptions - commanded topic and encoder topics
        self.create_subscription(Float64MultiArray, '/commanded', self.drive_callback, 10)
        self.create_subscription(Int32, '/left_encoder', self.left_encoder_callback, 10)
        self.create_subscription(Int32, '/right_encoder', self.right_encoder_callback, 10)
        
        # Subscription for laser scan (obstacle detection)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Obstacle detection state
        self.min_dis = float('inf')  # Minimum distance from laser scan

        # Publisher for odometry
        self.odom_pub = self.create_publisher(Pose2D, '/odom', 10)

        # Odometry state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.odom_prev_left_ticks = 0
        self.odom_prev_right_ticks = 0
        self.current_left_ticks = 0
        self.current_right_ticks = 0

        # Create timer for odometry publishing
        self.odom_timer = self.create_timer(PID_UPDATE_RATE, self.publish_odometry)
        
        self.get_logger().info('Motor controller initialized with FULLY CONCURRENT control')
        self.get_logger().info(f'PID update rate: {PID_UPDATE_RATE}s ({1/PID_UPDATE_RATE:.1f}Hz)')
        self.get_logger().info(f'PID gains: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}')
        self.get_logger().info(f'All 4 actuators running in independent threads')
        self.get_logger().info(f'Wheel radius: {WHEEL_RADIUS}m, Encoder TPR: {ENCODER_TICKS_PER_REV}')
        self.get_logger().info(f'Obstacle detection enabled with MIN_SAFE_DISTANCE: {MIN_SAFE_DISTANCE}m')

    def left_encoder_callback(self, msg):
        """Update current left encoder tick count"""
        self.current_left_ticks = msg.data
        self.motor_left.update_encoder(msg.data)

    def right_encoder_callback(self, msg):
        """Update current right encoder tick count"""
        self.current_right_ticks = msg.data
        self.motor_right.update_encoder(msg.data)

    def scan_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        # Filter out invalid ranges (inf and nan values)
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        
        # Calculate minimum distance from valid ranges
        if valid_ranges:
            self.min_dis = min(valid_ranges)
        else:
            self.min_dis = float('inf')  # No valid readings

    def publish_odometry(self):
        """
        Publish odometry at regular intervals.
        """
        # Get current stepper angles (thread-safe)
        left_angle = self.stepper_left.get_current_angle()
        right_angle = self.stepper_right.get_current_angle()
        
        # Update odometry
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

    def drive_callback(self, msg):
        """
        Process motor commands from /commanded topic.
        Expected format: [target_left_mps, target_right_mps, fl_angle, fr_angle]
        Speeds are in m/s, angles are ABSOLUTE TARGET POSITIONS in radians.
        
        All 4 actuators operate independently in their own threads.
        """
        data = msg.data
        if len(data) != 4:
            self.get_logger().error('Expected 4 values in msg.data')
            return
        
        # Extract values
        target_left_speed = -float(data[0])   # m/s
        target_right_speed = -float(data[1])  # m/s
        fl_angle = float(data[2])             # radians (absolute target)
        fr_angle = float(data[3])             # radians (absolute target)
        
        
        self.get_logger().info(
            f'New command - Speeds: L={target_left_speed:.3f}m/s, '
            f'R={target_right_speed:.3f}m/s | '
            f'Angles: FL={fl_angle:.3f}rad, FR={fr_angle:.3f}rad | '
            f'Min dist: {self.min_dis:.3f}m'
        )
        
        # Command steppers to move to target angles (in their own threads)
        self.stepper_left.move_to_angle(fl_angle)
        self.stepper_right.move_to_angle(fr_angle)

        # SAFETY CHECK: Obstacle detection. IT IS FINE TO BACK UP
        if target_left_speed < 0 and self.min_dis < MIN_SAFE_DISTANCE:
            self.get_logger().warn(
                f'OBSTACLE DETECTED! Min distance: {self.min_dis:.3f}m < {MIN_SAFE_DISTANCE}m - STOPPING'
            )
            # Stop all motors
            self.motor_left.set_target_speed(0.0)
            self.motor_right.set_target_speed(0.0)
            return
        
        # Set DC motor target speeds (applied in their own threads)
        self.motor_left.set_target_speed(target_left_speed)
        self.motor_right.set_target_speed(target_right_speed)
        

    def cleanup(self):
        """Stop motors and cleanup GPIO."""
        self.get_logger().info('Cleaning up motors...')
        
        # Stop all motors
        self.stepper_left.stop()
        self.stepper_right.stop()
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
