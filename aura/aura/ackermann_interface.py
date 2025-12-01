import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
from ackermann_msgs.msg import AckermannDriveStamped


class AckermannDriver(Node):
    def __init__(self):
        super().__init__('ackermann_driver')

        # These values do not matter much for Webots demo
        self.WHEELBASE = 1.0
        self.TRACK_WIDTH = 0.5
        self.WHEEL_RADIUS = 0.05

        robot = 'ackermann_vehicle'
        self.FL = f'/{robot}/front_left_steering_joint/set_position'
        self.FR = f'/{robot}/front_right_steering_joint/set_position'
        self.RL = f'/{robot}/rear_left_wheel_joint/set_velocity'
        self.RR = f'/{robot}/rear_right_wheel_joint/set_velocity'

        # Subscribe to the topic your teleop publishes
        self.sub = self.create_subscription(
            AckermannDriveStamped,
            '/cmd_ackermann',
            self.cmd_callback,
            10
        )

        # Publishers
        self.pub_fl = self.create_publisher(Float64, self.FL, 10)
        self.pub_fr = self.create_publisher(Float64, self.FR, 10)
        self.pub_rl = self.create_publisher(Float64, self.RL, 10)
        self.pub_rr = self.create_publisher(Float64, self.RR, 10)

        self.get_logger().info("Ackermann driver ready.")

    def cmd_callback(self, msg):
        speed = msg.drive.speed
        steer = msg.drive.steering_angle

        fl = Float64()
        fr = Float64()
        rl = Float64()
        rr = Float64()

        # Steering angles (simple, not true Ackermann)
        fl.data = steer
        fr.data = steer

        # Convert m/s → rad/s
        wheel_speed = speed / self.WHEEL_RADIUS
        rl.data = wheel_speed
        rr.data = wheel_speed

        self.pub_fl.publish(fl)
        self.pub_fr.publish(fr)
        self.pub_rl.publish(rl)
        self.pub_rr.publish(rr)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
