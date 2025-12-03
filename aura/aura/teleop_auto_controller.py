import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64MultiArray, String, Int32

from aura.constants import WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, MAX_STEERING_ANGLE, MAX_SPEED_LINEAR


class PIDController:
    """Simple PID controller for distance-based control"""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
    
    def compute(self, error, dt):
        """
        Compute PID output
        error: distance remaining (target - current)
        dt: time step
        """
        # Proportional term
        p = self.kp * error
        
        # Integral term (with windup protection)
        self.integral += error * dt
        self.integral = max(-10.0, min(10.0, self.integral))  # Limit integral windup
        i = self.ki * self.integral
        
        # Derivative term
        d = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        return p + i + d
    
    def reset(self):
        """Reset PID state"""
        self.prev_error = 0.0
        self.integral = 0.0


class AckermannDriveNode(Node):
    def __init__(self):
        super().__init__('ackermann_drive_node')

        # Publishers
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, '/cmd_ackermann', 10)
        self.commanded_pub = self.create_publisher(Float64MultiArray, '/commanded', 10)
        self.mode_pub = self.create_publisher(String, '/drive_mode', 10)

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.left_encoder_sub = self.create_subscription(Int32, '/left_encoder', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(Int32, '/right_encoder', self.right_encoder_callback, 10)

        # Controller button mapping
        self.button_auto = 4      # Button 4 for AUTO mode
        self.button_manual = 5    # Button 5 for MANUAL mode
        self.axis_speed = 1       # Left stick vertical
        self.axis_steer = 2       # Right stick horizontal

        # Mode state
        self.mode = "STOP"
        self.previous_mode = "STOP"

        # Encoder state
        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0
        self.encoder_baseline_left = 0
        self.encoder_baseline_right = 0
        self.ticks_per_rev = 600
        self.wheel_circumference = 2 * math.pi * WHEEL_RADIUS

        # Autonomous control
        self.auto_active = False
        self.auto_segment_queue = []
        self.current_segment = None
        self.segment_start_time = None

        # PID controllers for each wheel (distance-based)
        self.pid_left = PIDController(kp=-1.0, ki=0.0, kd=0.0)
        self.pid_right = PIDController(kp=-1.0, ki=0.0, kd=0.0)
        self.left_target_distance = 0.0
        self.right_target_distance = 0.0

        # Timers
        self.mode_timer = self.create_timer(0.1, self.publish_mode)
        self.auto_control_timer = self.create_timer(0.05, self.autonomous_control_loop)  # 20 Hz

        self.get_logger().info("Ackermann Drive Node Initialized")

    # ==================== ENCODER CALLBACKS ====================
    def left_encoder_callback(self, msg):
        self.left_encoder_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_encoder_ticks = msg.data

    def get_distance_traveled(self):
        """Calculate distance traveled since baseline was set (in meters)"""
        left_distance = ((self.left_encoder_ticks - self.encoder_baseline_left) / self.ticks_per_rev) * self.wheel_circumference
        right_distance = ((self.right_encoder_ticks - self.encoder_baseline_right) / self.ticks_per_rev) * self.wheel_circumference
        return left_distance, right_distance

    def reset_encoder_baseline(self):
        """Set current encoder values as the baseline for distance measurement"""
        self.encoder_baseline_left = self.left_encoder_ticks
        self.encoder_baseline_right = self.right_encoder_ticks
        # Reset PIDs
        self.pid_left.reset()
        self.pid_right.reset()

    # ==================== MODE MANAGEMENT ====================
    def publish_mode(self):
        """Publish current drive mode"""
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    def set_mode(self, new_mode):
        """Change mode and handle transitions"""
        if new_mode != self.mode:
            self.get_logger().info(f"Mode change: {self.mode} → {new_mode}")
            self.previous_mode = self.mode
            self.mode = new_mode

            # Stop motors on any mode transition
            if self.mode == "STOP":
                self.stop()
                self.auto_active = False
                self.auto_segment_queue.clear()
                self.current_segment = None
            elif self.mode == "AUTO" and self.previous_mode != "AUTO":
                # Starting AUTO mode - queue up demo segments
                self.start_autonomous_demo()

    # ==================== JOYSTICK CALLBACK ====================
    def joy_callback(self, joy_msg: Joy):
        # Determine mode based on buttons
        button_auto_pressed = len(joy_msg.buttons) > self.button_auto and joy_msg.buttons[self.button_auto]
        button_manual_pressed = len(joy_msg.buttons) > self.button_manual and joy_msg.buttons[self.button_manual]

        # Mode priority: MANUAL > AUTO > STOP
        if button_manual_pressed:
            self.set_mode("MANUAL")
        elif button_auto_pressed:
            self.set_mode("AUTO")
        else:
            self.set_mode("STOP")

        # Execute manual control if in MANUAL mode
        if self.mode == "MANUAL":
            self.manual_drive(joy_msg)

    # ==================== MANUAL CONTROL ====================
    def manual_drive(self, joy_msg: Joy):
        """Handle manual joystick control with Ackermann geometry"""
        # Forward speed
        v_center = joy_msg.axes[self.axis_speed] * MAX_SPEED_LINEAR

        # Steering input mapped to physical max steering angle
        steer_input = joy_msg.axes[self.axis_steer] * MAX_STEERING_ANGLE

        # Calculate wheel angles and speeds
        fl_angle, fr_angle, fl_speed, fr_speed = self.calculate_ackermann(v_center, steer_input)

        # Publish AckermannDriveStamped
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = v_center
        drive_msg.drive.steering_angle = steer_input
        self.ackermann_pub.publish(drive_msg)

        # Publish commanded wheel speeds and angles
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [fl_speed, fr_speed, fl_angle, fr_angle]
        self.commanded_pub.publish(cmd_msg)

    # ==================== ACKERMANN GEOMETRY ====================
    def calculate_ackermann(self, v_center, steer_input):
        """
        Calculate individual wheel angles and speeds based on Ackermann geometry
        Returns: fl_angle, fr_angle, fl_speed, fr_speed
        """
        if abs(steer_input) > 1e-6:
            R_turn = WHEELBASE / math.tan(abs(steer_input))

            # Compute left and right wheel steering angles
            left_angle_abs = math.atan(WHEELBASE / (R_turn - TRACK_WIDTH / 2))
            right_angle_abs = math.atan(WHEELBASE / (R_turn + TRACK_WIDTH / 2))

            # Compute wheel linear speeds
            v_left = v_center * (R_turn - TRACK_WIDTH / 2) / R_turn
            v_right = v_center * (R_turn + TRACK_WIDTH / 2) / R_turn

            # Assign angles and speeds based on turn direction
            if steer_input > 0:  # left turn
                fl_angle = left_angle_abs
                fr_angle = right_angle_abs
                fl_speed = v_left
                fr_speed = v_right
            else:  # right turn
                fl_angle = -right_angle_abs
                fr_angle = -left_angle_abs
                fl_speed = v_right
                fr_speed = v_left
        else:
            # Straight driving
            fl_angle = fr_angle = 0.0
            fl_speed = fr_speed = v_center

        return fl_angle, fr_angle, fl_speed, fr_speed

    # ==================== AUTONOMOUS CONTROL ====================
    def start_autonomous_demo(self):
        """Queue up the autonomous demo segments"""
        self.get_logger().info("=== AUTONOMOUS DEMO START ===")
        
        feet_to_meters = 0.3048
        target_distance = 0.5 * feet_to_meters  # 5 feet
        
        # Queue segments: each segment is a dict with type, params
        self.auto_segment_queue = [
            {
                'type': 'drive',
                'speed': 10.5,
                'steer': 0.0,
                'distance': target_distance,
                'name': 'Segment 1: 5 feet forward'
            },
            {
                'type': 'pause',
                'duration': 2.0,
                'name': 'Pause'
            },
            {
                'type': 'drive',
                'speed': 20.5,
                'steer': 0.2,
                'distance': target_distance,
                'name': 'Segment 2: 5 feet at angle'
            }
        ]
        
        self.auto_active = True
        self.load_next_segment()

    def load_next_segment(self):
        """Load the next segment from the queue"""
        if not self.auto_segment_queue:
            self.get_logger().info("=== AUTONOMOUS DEMO COMPLETE ===")
            self.auto_active = False
            self.current_segment = None
            self.stop()
            return
        
        self.current_segment = self.auto_segment_queue.pop(0)
        self.get_logger().info(f"{self.current_segment['name']}")
        
        if self.current_segment['type'] == 'drive':
            self.reset_encoder_baseline()
            # Calculate target distance for each wheel based on Ackermann geometry
            self.calculate_wheel_targets(
                self.current_segment['distance'],
                self.current_segment['steer']
            )
        elif self.current_segment['type'] == 'pause':
            self.segment_start_time = self.get_clock().now()

    def calculate_wheel_targets(self, center_distance, steer_angle):
        """
        Calculate individual wheel target distances based on Ackermann geometry.
        For turning, inner wheel travels less, outer wheel travels more.
        """
        if abs(steer_angle) > 1e-6:
            # Calculate turn radius at vehicle center
            R_turn = WHEELBASE / math.tan(abs(steer_angle))
            
            # Calculate distance for each wheel
            left_distance = center_distance * (R_turn - TRACK_WIDTH / 2) / R_turn
            right_distance = center_distance * (R_turn + TRACK_WIDTH / 2) / R_turn
            
            # Assign based on turn direction
            if steer_angle > 0:  # left turn
                self.left_target_distance = left_distance
                self.right_target_distance = right_distance
            else:  # right turn
                self.left_target_distance = right_distance
                self.right_target_distance = left_distance
        else:
            # Straight - both wheels travel same distance
            self.left_target_distance = center_distance
            self.right_target_distance = center_distance
        
        self.get_logger().info(f"Target distances - Left: {self.left_target_distance:.3f}m, Right: {self.right_target_distance:.3f}m")

    def autonomous_control_loop(self):
        """Non-blocking timer callback for autonomous control"""
        if self.mode != "AUTO" or not self.auto_active or self.current_segment is None:
            return

        if self.current_segment['type'] == 'drive':
            # Execute driving segment with PID control
            target_speed = self.current_segment['speed']
            steer = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, self.current_segment['steer']))
            
            # Get current distance traveled by each wheel
            left_distance, right_distance = self.get_distance_traveled()
            
            # Calculate error for each wheel
            left_error = self.left_target_distance - abs(left_distance)
            right_error = self.right_target_distance - abs(right_distance)
            
            # PID output gives velocity adjustment
            left_speed_adjustment = self.pid_left.compute(left_error, 0.05)  # dt = 0.05s (20Hz)
            right_speed_adjustment = self.pid_right.compute(right_error, 0.05)
            
            # Calculate base wheel speeds from Ackermann geometry
            _, _, fl_speed_base, fr_speed_base = self.calculate_ackermann(target_speed, steer)
            
            # Apply PID adjustments (limit to reasonable range)
            fl_speed = max(-MAX_SPEED_LINEAR, min(MAX_SPEED_LINEAR, fl_speed_base + left_speed_adjustment))
            fr_speed = max(-MAX_SPEED_LINEAR, min(MAX_SPEED_LINEAR, fr_speed_base + right_speed_adjustment))
            
            # Get steering angles
            fl_angle, fr_angle, _, _ = self.calculate_ackermann(target_speed, steer)
            
            # Log progress - simpler output showing raw encoder values
            left_target_ticks = (self.left_target_distance / self.wheel_circumference) * self.ticks_per_rev
            right_target_ticks = (self.right_target_distance / self.wheel_circumference) * self.ticks_per_rev

            # Log progress - show encoder values vs expected encoder positions
            self.get_logger().info(
                f"Left encoder: {self.left_encoder_ticks} (target: {int(left_target_ticks)} ticks) | "
                f"Right encoder: {self.right_encoder_ticks} (target: {int(right_target_ticks)} ticks)"
            )
            
            # Check if both wheels reached target (within tolerance)
            tolerance = 0.02  # 2cm tolerance
            if abs(left_error) < tolerance and abs(right_error) < tolerance:
                self.get_logger().info(f"Segment complete! L: {abs(left_distance):.3f}m, R: {abs(right_distance):.3f}m")
                self.stop()
                self.load_next_segment()
                return
            
            # Publish commands
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [fl_speed, fr_speed, fl_angle, fr_angle]
            self.commanded_pub.publish(cmd_msg)
            
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = (fl_speed + fr_speed) / 2.0
            drive_msg.drive.steering_angle = steer
            self.ackermann_pub.publish(drive_msg)
        
        elif self.current_segment['type'] == 'pause':
            # Non-blocking pause
            elapsed = (self.get_clock().now() - self.segment_start_time).nanoseconds / 1e9
            if elapsed >= self.current_segment['duration']:
                self.load_next_segment()

    def stop(self):
        """Send stop command to all wheels"""
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0]
        self.commanded_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()