import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

class AckermannTeleop(Node):
    def __init__(self):
        super().__init__('ackermann_teleop')

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/cmd_ackermann', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Axes/buttons (depends on controller)
        self.axis_speed = 1    # left stick vertical
        self.axis_steer = 2    # right stick horizontal
        self.enable_button = 5 # right bumper

        self.speed_scale = 1.0
        self.steer_scale = 0.5

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) > self.enable_button and msg.buttons[self.enable_button]:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = msg.axes[self.axis_speed] * self.speed_scale
            drive_msg.drive.steering_angle = msg.axes[self.axis_steer] * self.steer_scale
            self.drive_pub.publish(drive_msg)
            self.get_logger().info(f"Speed: {drive_msg.drive.speed:.2f}, Steer: {drive_msg.drive.steering_angle:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = AckermannTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
