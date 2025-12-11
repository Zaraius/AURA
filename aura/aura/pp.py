import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

class ReactiveNavigator(Node):
    def __init__(self):
        super().__init__('pp')

        # --- TUNING PARAMETERS ---
        self.GOAL_X = 2.0           # Target X coordinate (meters)
        self.GOAL_Y = 0.0           # Target Y coordinate
        self.K_ATTRACT = 1.0        # How strongly to pull to goal
        self.K_REPULSE = 0.5        # How strongly to push away from obstacles
        self.REPULSION_DIST = 0.8   # Obstacles further than this don't matter (meters)
        self.MAX_SPEED = 0.3        # Max linear speed (m/s)
        self.MAX_TURN = 1.0         # Max angular speed (rad/s)

        # --- SUBSCRIBERS ---
        # Listen to Lidar (Depth at angles)
        self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Listen to position
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publish command
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.scan_data = None

        # Loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"Reactive Navigator Started. Goal: ({self.GOAL_X}, {self.GOAL_Y})")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Convert Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        self.scan_data = msg

    def control_loop(self):
        if self.scan_data is None:
            return

        # --- 1. ATTRACTIVE FORCE (Pull to Goal) ---
        dx = self.GOAL_X - self.robot_x
        dy = self.GOAL_Y - self.robot_y
        dist_to_goal = math.hypot(dx, dy)

        # Stop if close
        if dist_to_goal < 0.15:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("GOAL REACHED!")
            return

        # Force vector towards goal
        force_x = self.K_ATTRACT * dx
        force_y = self.K_ATTRACT * dy

        # --- 2. REPULSIVE FORCE (Push from Obstacles) ---
        # Lidar gives us an array of ranges. We check them all.
        ranges = self.scan_data.ranges
        angle_min = self.scan_data.angle_min
        angle_inc = self.scan_data.angle_increment

        for i, r in enumerate(ranges):
            # Filter bad data (inf or 0.0)
            if r <= 0.0 or r > self.REPULSION_DIST or math.isinf(r):
                continue

            # Calculate the angle of this specific ray
            angle = angle_min + (i * angle_inc) + self.robot_yaw
            
            # Create a repulsion vector pointing AWAY from the obstacle
            # Formula: Force = K * (1/dist)
            repulsion_strength = self.K_REPULSE * (1.0 / r)
            
            # The force pushes in the opposite direction of the reading
            force_x -= repulsion_strength * math.cos(angle)
            force_y -= repulsion_strength * math.sin(angle)

        # --- 3. RESULTANT MOTION ---
        # The final "force" tells us the best x,y direction to move
        target_heading = math.atan2(force_y, force_x)
        
        # Calculate error between where we want to go and where we are facing
        heading_error = target_heading - self.robot_yaw
        
        # Normalize angle to [-pi, pi]
        while heading_error > math.pi: heading_error -= 2 * math.pi
        while heading_error < -math.pi: heading_error += 2 * math.pi

        # Create Twist Command
        twist = Twist()
        
        # Turn to face the resultant force
        twist.angular.z = 2.0 * heading_error
        # Clamp turning speed
        twist.angular.z = max(min(twist.angular.z, self.MAX_TURN), -self.MAX_TURN)

        # Drive forward only if we are roughly facing the right way
        if abs(heading_error) < 1.0:
            twist.linear.x = self.MAX_SPEED
        else:
            twist.linear.x = 0.0 # Pivot in place first

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
