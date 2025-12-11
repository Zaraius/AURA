from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class STATE(Enum):
    MANUAL = "MANUAL"
    AUTO = "AUTO"
    STOP = "STOP"
    PP = "PP"

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        self.state = STATE.AUTO

        self.pub = self.create_publisher(String, "/drive_mode", 10)
        self.create_timer(1.0, self.publish_state)

    def set_state(self, new_state: STATE):
        self.state = new_state
        self.publish_state()
        self.get_logger().info(f"Mode changed → {new_state.value}")

    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
