import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64MultiArray, String, Int32
from geometry_msgs.msg import Point, Pose2D # ADDED Pose2D here

# --- NEW IMPORTS FOR PATH PLANNING ---
from nav_msgs.msg import OccupancyGrid # Removed Odometry from here since we use Pose2D
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
import numpy as np

from aura.constants import WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, MAX_STEERING_ANGLE, MAX_SPEED_LINEAR

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
        self.target_sub = self.create_subscription(Point, '/target', self.target_callback, 10)
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Pose2D, '/odom', self.odom_callback, 10)

        # Controller button mapping
        self.button_auto = 4      # Button 4 for AUTO/FOLLOW mode
        self.button_manual = 5    # Button 5 for MANUAL mode
        self.button_pp = 1        # Button 1 (A) for Path Planning Mode
        self.axis_speed = 1       # Left stick vertical
        self.axis_steer = 2       # Right stick horizontal

        # Mode state
        self.mode = "STOP"
        self.previous_mode = "STOP"

        # Encoder state (Kept for logging, but not used for PID anymore)
        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0
        self.encoder_baseline_left = 0
        self.encoder_baseline_right = 0
        self.ticks_per_rev = 600
        self.wheel_circumference = 2 * math.pi * WHEEL_RADIUS

        # Follower Vision State
        self.latest_target = None
        self.last_target_time = 0.0
        
        # Follower Tuning Parameters
        self.target_offset = 1.0       # Maintain 0.5 meter distance
        self.follow_kp_speed = 30     # P-Gain for Speed
        self.follow_kp_steer = 5.0     # P-Gain for Steering
        self.deadband_dist = 0.1       # 10cm tolerance
        self.vision_timeout = 0.5      # Stop if no target seen for 0.5s

        # --- PATH PLANNING STATE ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.map_data = None
        self.map_info = None
        self.path = []
        self.pp_goal_x = 0.5  # PLACEHOLDER GOAL X (Meters)
        self.pp_goal_y = 0.0  # PLACEHOLDER GOAL Y (Meters)
        self.pp_lookahead = 0.5
        self.pp_goal_tolerance = 0.15

        # Timers
        self.mode_timer = self.create_timer(0.1, self.publish_mode)
        self.follow_control_timer = self.create_timer(0.05, self.follow_control_loop)  # 20 Hz
        self.pp_control_timer = self.create_timer(0.1, self.path_planning_loop)       # 10 Hz (PP Mode)

        self.get_logger().info("Ackermann Drive Node Initialized (Follow + PP Mode Ready)")

    # ==================== ENCODER CALLBACKS ====================
    def left_encoder_callback(self, msg):
        self.left_encoder_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_encoder_ticks = msg.data

    # ==================== VISION CALLBACK ====================
    def target_callback(self, msg):
        self.latest_target = msg
        self.last_target_time = time.time()

    # ==================== PATH PLANNING CALLBACKS ====================
    def map_callback(self, msg):
        self.map_info = msg.info
        # Convert ROS 1D array to numpy 2D array
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    # UPDATED: Callback for Pose2D
    def odom_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_yaw = msg.theta # Pose2D gives theta directly, no quaternion math needed

    # ==================== MODE MANAGEMENT ====================
    def publish_mode(self):
        """Publish current drive mode"""
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    def set_mode(self, new_mode):
        """Change mode and handle transitions"""
        if new_mode != self.mode:
            self.get_logger().info(f"Mode change: {self.mode} -> {new_mode}")
            self.previous_mode = self.mode
            self.mode = new_mode

            # Stop motors on any mode transition
            if self.mode == "STOP":
                self.stop()
            elif self.mode == "AUTO":
                # Reset buffers or prepare for following if needed
                self.get_logger().info("Follow Mode Activated")
            elif self.mode == "PP":
                self.get_logger().info("Path Planning Mode Activated")
                self.generate_path() # Generate path once when entering PP mode

    # ==================== JOYSTICK CALLBACK ====================
    def joy_callback(self, joy_msg: Joy):
        # Determine mode based on buttons
        button_auto_pressed = len(joy_msg.buttons) > self.button_auto and joy_msg.buttons[self.button_auto]
        button_manual_pressed = len(joy_msg.buttons) > self.button_manual and joy_msg.buttons[self.button_manual]
        button_pp_pressed = len(joy_msg.buttons) > self.button_pp and joy_msg.buttons[self.button_pp]

        # Mode priority: MANUAL > AUTO (FOLLOW) > PP > STOP
        if button_manual_pressed:
            self.set_mode("MANUAL")
        elif button_auto_pressed:
            self.set_mode("AUTO") # Reusing AUTO string for Follow mode logic
        elif button_pp_pressed:
            self.set_mode("PP")
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

    # ==================== FOLLOWER CONTROL ====================
    def follow_control_loop(self):
        """Non-blocking timer callback for Vision Following"""
        # Only run if we are in AUTO mode
        if self.mode != "AUTO":
            return

        # 1. Safety Check: Have we heard from the camera recently?
        if (time.time() - self.last_target_time) > self.vision_timeout:
            # Timeout - stop the robot
            self.stop()
            return

        target = self.latest_target
        if target is None or target.z == 0.0:
            self.stop()
            return

        # --- STEERING CONTROL (P-Controller) ---
        # Image Center = 320. 
        # Left (x < 320) -> Error positive -> Turn Left
        # Right (x > 320) -> Error negative -> Turn Right
        steer_error = -(320.0 - target.x) / 320.0 # invert it because camera is mounted upside down
        cmd_steer = steer_error * self.follow_kp_steer

        # --- SPEED CONTROL (P-Controller with Offset) ---
        # Target Offset = 1.0m
        # If z = 3.0m -> Error = 2.0 -> Speed Positive (Forward)
        # If z = 0.5m -> Error = -0.5 -> Speed Negative (Backward)
        dist_error = target.z - self.target_offset
        
        # Deadband to prevent jitter when standing still
        if abs(dist_error) < self.deadband_dist:
            cmd_speed = 0.0
        else:
            cmd_speed = dist_error * self.follow_kp_speed

        # --- CLAMPING ---
        cmd_speed = max(-MAX_SPEED_LINEAR, min(MAX_SPEED_LINEAR, cmd_speed))
        cmd_steer = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, cmd_steer))

        # --- EXECUTE ---
        # Calculate individual wheel params
        fl_angle, fr_angle, fl_speed, fr_speed = self.calculate_ackermann(cmd_speed, cmd_steer)

        # Publish commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [fl_speed, fr_speed, fl_angle, fr_angle]
        self.commanded_pub.publish(cmd_msg)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(cmd_speed)
        drive_msg.drive.steering_angle = float(cmd_steer)
        self.ackermann_pub.publish(drive_msg)

    # ==================== PATH PLANNING LOGIC ====================
    def generate_path(self):
        """ Runs A* one time to find path from current Odom to Placeholder Goal """
        if self.map_data is None:
            self.get_logger().warn("PP: No Map Data! Ensure SLAM is running.")
            return

        start_grid = self.world_to_grid(self.current_x, self.current_y)
        end_grid = self.world_to_grid(self.pp_goal_x, self.pp_goal_y)

        # 0 = Free, 100 = Occupied. Library expects 1=Walkable, 0=Obstacle
        # We treat anything > 50 probability as an obstacle
        walkable_matrix = np.where(self.map_data > 50, 0, 1) 
        grid = Grid(matrix=walkable_matrix)

        # Bounds check
        if not (0 <= start_grid[0] < self.map_info.width and 0 <= start_grid[1] < self.map_info.height):
             self.get_logger().warn("PP: Start position outside map bounds.")
             return

        start_node = grid.node(start_grid[0], start_grid[1])
        end_node = grid.node(end_grid[0], end_grid[1])

        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path_nodes, runs = finder.find_path(start_node, end_node, grid)

        if len(path_nodes) < 1:
            self.get_logger().warn("PP: No Path Found! Obstacle might be blocking goal.")
            self.path = []
            return

        # Convert back to world coords
        self.path = [self.grid_to_world(p.x, p.y) for p in path_nodes]
        self.get_logger().info(f"PP: Path Generated with {len(self.path)} steps.")

    def path_planning_loop(self):
        """ The loop that runs when mode == PP """
        if self.mode != "PP":
            return

        if not self.path:
            self.stop()
            return

        # 1. Pure Pursuit Logic
        # Find lookahead point
        target_x, target_y = self.path[-1] # Default to end
        
        # Check if we reached goal
        dist_to_final = math.hypot(target_x - self.current_x, target_y - self.current_y)
        if dist_to_final < self.pp_goal_tolerance:
            self.get_logger().info("PP: Goal Reached")
            self.stop()
            self.path = [] # Clear path
            self.set_mode("STOP")
            return

        # Search for lookahead point
        for px, py in self.path:
            dist = math.hypot(px - self.current_x, py - self.current_y)
            if dist > self.pp_lookahead:
                target_x, target_y = px, py
                break

        # 2. Calculate Steering
        angle_to_target = math.atan2(target_y - self.current_y, target_x - self.current_x)
        
        # Normalize angle error (-pi to pi)
        angle_error = angle_to_target - self.current_yaw
        while angle_error > math.pi: angle_error -= 2.0 * math.pi
        while angle_error < -math.pi: angle_error += 2.0 * math.pi

        # 3. Simple Control
        # P-Controller for steering
        cmd_steer = angle_error * 1.5 
        cmd_speed = MAX_SPEED_LINEAR * 0.5 # Half speed for safety

        # Clamp
        cmd_steer = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, cmd_steer))

        # 4. Obstacle Safety Check (Front of robot)
        # Look 0.5m ahead
        front_x = self.current_x + 0.5 * math.cos(self.current_yaw)
        front_y = self.current_y + 0.5 * math.sin(self.current_yaw)
        gx, gy = self.world_to_grid(front_x, front_y)
        
        if self.map_info and (0 <= gx < self.map_info.width) and (0 <= gy < self.map_info.height):
            # Access map data (row-major order usually y * width + x, but here numpy 2d is [y,x])
            if self.map_data[gy, gx] > 50:
                self.get_logger().warn("PP: Obstacle Ahead! Stopping.")
                self.stop()
                return

        self.get_logger().info(f"x: {self.current_x}, y: {self.current_y}, yaw: {self.current_yaw}, cmd_speed: {cmd_speed}, cmd_steer: {cmd_steer}")
        # 5. Execute
        fl_angle, fr_angle, fl_speed, fr_speed = self.calculate_ackermann(cmd_speed, cmd_steer)

        cmd_msg = Float64MultiArray()
        cmd_msg.data = [fl_speed, fr_speed, fl_angle, fr_angle]
        self.commanded_pub.publish(cmd_msg)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(cmd_speed)
        drive_msg.drive.steering_angle = float(cmd_steer)
        self.ackermann_pub.publish(drive_msg)

    # ==================== HELPERS ====================
    def world_to_grid(self, wx, wy):
        if self.map_info is None: return (0,0)
        gx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        if self.map_info is None: return (0,0)
        wx = (gx * self.map_info.resolution) + self.map_info.origin.position.x
        wy = (gy * self.map_info.resolution) + self.map_info.origin.position.y
        return (wx, wy)

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
