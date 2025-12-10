import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
import math

# Left encoder pins
A_PIN = 23
B_PIN = 24

# Right encoder pins
C_PIN = 25
D_PIN = 8


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')

        # Robot parameters (adjust these to match your robot)
        self.declare_parameter('wheel_radius', 0.033)  # meters
        self.declare_parameter('wheel_base', 0.16)     # meters (distance between wheels)
        self.declare_parameter('counts_per_revolution', 360)  # encoder ticks per wheel revolution
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.counts_per_rev = self.get_parameter('counts_per_revolution').value
        
        # Distance per encoder count
        self.distance_per_count = (2 * math.pi * self.wheel_radius) / self.counts_per_rev

        # Encoder positions
        self.left_pos = 0
        self.right_pos = 0
        
        # Previous encoder positions for odometry calculation
        self.prev_left_pos = 0
        self.prev_right_pos = 0

        # Last states
        self.last_a = 0
        self.last_c = 0

        # Odometry state (starting at origin)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.setup(C_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(D_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Poll at 1 kHz
        self.poll_timer = self.create_timer(0.001, self.poll_encoders)

        # Publish at 50 Hz
        self.pub_left = self.create_publisher(Int32, '/left_encoder', 10)
        self.pub_right = self.create_publisher(Int32, '/right_encoder', 10)
        self.pub_odom = self.create_publisher(Pose2D, '/odom', 10)

        self.pub_timer = self.create_timer(0.02, self.publish_positions)

    def poll_encoders(self):
        # ----- Left encoder -----
        a = GPIO.input(A_PIN)
        b = GPIO.input(B_PIN)

        if a != self.last_a:  # edge detected on A
            if a == b:
                self.left_pos += 1
            else:
                self.left_pos -= 1
        self.last_a = a

        # ----- Right encoder -----
        c = GPIO.input(C_PIN)
        d = GPIO.input(D_PIN)

        if c != self.last_c:  # edge detected on C
            if c == d:
                self.right_pos += 1
            else:
                self.right_pos -= 1
        self.last_c = c

    def update_odometry(self):
        # Calculate change in encoder counts
        delta_left = self.left_pos - self.prev_left_pos
        delta_right = self.right_pos - self.prev_right_pos
        
        # Update previous positions
        self.prev_left_pos = self.left_pos
        self.prev_right_pos = self.right_pos
        
        # Convert encoder counts to distances
        left_distance = delta_left * self.distance_per_count
        right_distance = delta_right * self.distance_per_count
        
        # Calculate robot movement using differential drive kinematics
        distance_center = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base
        
        # Update pose using midpoint integration
        if abs(delta_theta) < 1e-6:
            # Straight line motion
            delta_x = distance_center * math.cos(self.theta)
            delta_y = distance_center * math.sin(self.theta)
        else:
            # Arc motion
            radius = distance_center / delta_theta
            delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
        
        # Update odometry state
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def publish_positions(self):
        # Update odometry first
        self.update_odometry()
        
        # Publish encoder counts
        msg_left = Int32()
        msg_left.data = self.left_pos
        self.pub_left.publish(msg_left)

        msg_right = Int32()
        msg_right.data = self.right_pos
        self.pub_right.publish(msg_right)
        
        # Publish odometry
        odom_msg = Pose2D()
        odom_msg.x = self.x
        odom_msg.y = self.y
        odom_msg.theta = self.theta
        self.pub_odom.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()