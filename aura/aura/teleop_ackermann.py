import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class AckermannTeleop(Node):
    def __init__(self):
        super().__init__('ackermann_teleop')

        # Publishers
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, '/cmd_ackermann', 10)
        self.commanded_pub = self.create_publisher(Float64MultiArray, '/commanded', 10)

        # Joystick subscription
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Controller mapping (adjust for your joystick)
        self.axis_speed = 1    # Left stick vertical
        self.axis_steer = 3    # Right stick horizontal
        self.enable_button = 5 # Right bumper

        # Scaling factors
        self.speed_scale = 0.025
        self.steer_scale = -1.1071487177940904 / 90

        # Robot constants
        self.HALF_DISTANCE_BETWEEN_WHEELS = 0.25
        self.WHEEL_RADIUS = 0.15
        self.WHEELBASE = 0.5   # distance between front and rear axles
        self.TRACK_WIDTH = 0.5 # distance between left/right wheels

        self.enabled = False
    
    def joy_callback(self, msg: Joy):
        # Only act when enable button (RB) is pressed
        if len(msg.buttons) > self.enable_button and msg.buttons[self.enable_button]:
            self.enabled = True
            # Ackermann command
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = msg.axes[self.axis_speed] * self.speed_scale
            drive_msg.drive.steering_angle = msg.axes[self.axis_steer] * self.steer_scale
            self.ackermann_pub.publish(drive_msg)

            # Convert to differential drive velocities
            forward_speed = drive_msg.drive.speed
            angular_speed = drive_msg.drive.steering_angle

            command_motor_left = (forward_speed - angular_speed * self.HALF_DISTANCE_BETWEEN_WHEELS) / self.WHEEL_RADIUS
            command_motor_right = (forward_speed + angular_speed * self.HALF_DISTANCE_BETWEEN_WHEELS) / self.WHEEL_RADIUS
            if angular_speed != 0:
                turn_radius = forward_speed / angular_speed
                angle_inner = math.atan(self.WHEELBASE / (turn_radius - self.TRACK_WIDTH/2))
                angle_outer = math.atan(self.WHEELBASE / (turn_radius + self.TRACK_WIDTH/2))
                if angular_speed > 0:  # turning left
                    fl_angle = angle_inner
                    fr_angle = angle_outer
                else:  # turning right
                    fl_angle = -angle_outer
                    fr_angle = -angle_inner
            else:
                fl_angle = 0.0
                fr_angle = 0.0
            msg = Float64MultiArray()
            msg.data = [command_motor_left, command_motor_right, fl_angle, fr_angle]
            self.commanded_pub.publish(msg)
            # Print useful info
            self.get_logger().info(
                f"Speed: {forward_speed:.2f}, Steer: {angular_speed:.2f}, "
                f"L_wheel: {command_motor_left:.2f}, R_wheel: {command_motor_right:.2f}"
            )
            return
        if self.enabled:
            self.enabled = False  # reset

            msg_out = Float64MultiArray()
            msg_out.data = [0.0, 0.0, 0.0, 0.0]
            self.commanded_pub.publish(msg_out)

            self.get_logger().info("Commanded STOP after release")

def main(args=None):
    rclpy.init(args=args)
    node = AckermannTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
