import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64MultiArray, String

from aura.constants import WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, MAX_STEERING_ANGLE

class AckermannTeleop(Node):
    def __init__(self):
        super().__init__('ackermann_teleop')

        # Publishers
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, '/cmd_ackermann', 10)
        self.commanded_pub = self.create_publisher(Float64MultiArray, '/commanded', 10)

        # Joystick subscription
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.mode = "STOP"
        self.mode_sub = self.create_subscription(String, "/drive_mode", self.mode_callback, 10)
        # Controller mapping (adjust for your joystick)
        self.axis_speed = 1    # Left stick vertical
        self.axis_steer = 3    # Right stick horizontal
        self.enable_button = 5 # Right bumper

        # Scaling factors
        self.speed_scale = 1.0  # m/s scaling
        self.enabled = False
    
    def joy_callback(self, joy_msg: Joy):
        # Only act when enable button (RB) is pressed
        if len(joy_msg.buttons) > self.enable_button and joy_msg.buttons[self.enable_button] and self.mode=="MANUAL":
            self.enabled = True

            # Forward speed
            v_center = joy_msg.axes[self.axis_speed] * self.speed_scale

            # Steering input mapped to physical max steering angle
            steer_input = joy_msg.axes[self.axis_steer] * MAX_STEERING_ANGLE

            if abs(steer_input) > 1e-6:
                R_turn = WHEELBASE / math.tan(abs(steer_input))

                # Compute left and right wheel steering angles relative to straight ahead
                left_angle_abs = math.atan(WHEELBASE / (R_turn - TRACK_WIDTH / 2))
                right_angle_abs = math.atan(WHEELBASE / (R_turn + TRACK_WIDTH / 2))

                # Compute wheel linear speeds
                v_left = v_center * (R_turn - TRACK_WIDTH / 2) / R_turn
                v_right = v_center * (R_turn + TRACK_WIDTH / 2) / R_turn

                # Assign left/right angles and speeds depending on turn direction
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
                # Straight
                fl_angle = fr_angle = 0.0
                fl_speed = fr_speed = v_center

            # Convert linear speed to wheel angular velocity
            fl_speed /= WHEEL_RADIUS
            fr_speed /= WHEEL_RADIUS

            # Publish AckermannDriveStamped for controllers
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = v_center
            drive_msg.drive.steering_angle = steer_input
            self.ackermann_pub.publish(drive_msg)

            # Publish commanded wheel speeds and angles
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [fl_speed, fr_speed, fl_angle, fr_angle]
            self.commanded_pub.publish(cmd_msg)

            self.get_logger().info(
                f"v_center: {v_center:.2f}, steer_input: {steer_input:.2f}, "
                f"FL: angle {fl_angle:.2f} rad, speed {fl_speed:.2f} rad/s, "
                f"FR: angle {fr_angle:.2f} rad, speed {fr_speed:.2f} rad/s"
            )
            return

        # Stop when enable button released
        if self.enabled:
            self.enabled = False
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0, 0.0, 0.0, 0.0]
            self.commanded_pub.publish(stop_msg)
            self.get_logger().info("Commanded STOP after release")
        
        
    def mode_callback(self, msg):
        self.mode = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = AckermannTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()