import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class AckermannDriver(Node):
    def __init__(self):
        super().__init__('ackermann_driver')

        # === 1. TO-DO: UPDATE THESE VALUES ===
        # Get these from your URDF or Webots robot definition
        self.WHEELBASE = 1.0  # meters
        self.TRACK_WIDTH = 0.5  # meters

        # These are the topic names the WebotsController creates.
        # Use `ros2 topic list` to find your actual topic names.
        robot_name = 'ackermann_vehicle'
        self.FL_STEER_TOPIC = f'/{robot_name}/front_left_steering_joint/set_position'
        self.FR_STEER_TOPIC = f'/{robot_name}/front_right_steering_joint/set_position'
        self.RL_WHEEL_TOPIC = f'/{robot_name}/rear_left_wheel_joint/set_velocity'
        self.RR_WHEEL_TOPIC = f'/{robot_name}/rear_right_wheel_joint/set_velocity'
        # =======================================

        # Subscription to /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publishers for each motor
        self.fl_steer_pub = self.create_publisher(Float64, self.FL_STEER_TOPIC, 10)
        self.fr_steer_pub = self.create_publisher(Float64, self.FR_STEER_TOPIC, 10)
        self.rl_wheel_pub = self.create_publisher(Float64, self.RL_WHEEL_TOPIC, 10)
        self.rr_wheel_pub = self.create_publisher(Float64, self.RR_WHEEL_TOPIC, 10)

        self.get_logger().info("Ackermann driver ready.")

    def cmd_vel_callback(self, msg: Twist):
        speed = msg.linear.x
        steering_rate = msg.angular.z

        # Create messages
        fl_steer_msg = Float64()
        fr_steer_msg = Float64()
        rl_wheel_msg = Float64()
        rr_wheel_msg = Float64()

        if steering_rate == 0:
            # Going straight
            fl_steer_msg.data = 0.0
            fr_steer_msg.data = 0.0
        else:
            # Calculate turning radius
            turn_radius = speed / steering_rate

            # Calculate inner and outer steering angles (Ackermann formula)
            # Inner wheel
            angle_inner = math.atan(self.WHEELBASE / (turn_radius - self.TRACK_WIDTH / 2.0))
            # Outer wheel
            angle_outer = math.atan(self.WHEELBASE / (turn_radius + self.TRACK_WIDTH / 2.0))

            if steering_rate > 0:  # Turning left
                fl_steer_msg.data = angle_inner
                fr_steer_msg.data = angle_outer
            else:  # Turning right
                fl_steer_msg.data = -angle_outer  # Negative for right turn
                fr_steer_msg.data = -angle_inner # Negative for right turn

        # Set wheel speeds (simple, non-differential)
        # Webots motors often expect rad/s. You may need to convert m/s
        # (speed) to rad/s based on your wheel_radius.
        # For simplicity, we just pass the linear speed. Adjust as needed.
        # Example: speed_rad_s = speed / WHEEL_RADIUS
        rl_wheel_msg.data = speed * 10.0 # Adjust this multiplier
        rr_wheel_msg.data = speed * 10.0 # Adjust this multiplier

        # Publish all commands
        self.fl_steer_pub.publish(fl_steer_msg)
        self.fr_steer_pub.publish(fr_steer_msg)
        self.rl_wheel_pub.publish(rl_wheel_msg)
        self.rr_wheel_pub.publish(rr_wheel_msg)

        # Add this to the BOTTOM of ackermann_driver.py

def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()