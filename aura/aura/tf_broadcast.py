
#!/usr/bin/env python3
# broadcast /odom to TF without affecting other pieces of code
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        # TF broadcaster
        self.br = TransformBroadcaster(self)

        # Subscribe to your Pose2D odometry
        self.create_subscription(
            Pose2D,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg: Pose2D):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Position
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # Convert yaw → quaternion
        qz = math.sin(msg.theta / 2.0)
        qw = math.cos(msg.theta / 2.0)

        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        # Publish TF
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
