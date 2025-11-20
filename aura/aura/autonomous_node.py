import rclpy
import math
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from aura.constants import WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, MAX_STEERING_ANGLE


class AutonomousNode(Node):
    def __init__(self):
        super().__init__("autonomous_node")

        # Mode handling
        self.mode = "STOP"
        self.mode_sub = self.create_subscription(
            String, "/drive_mode", self.mode_callback, 10
        )

        # Publisher to motor controller
        self.commanded_pub = self.create_publisher(
            Float64MultiArray, "/commanded", 10
        )

        self.get_logger().info("Autonomous Node Initialized.")

    # --------------------
    # MODE CALLBACK
    # --------------------
    def mode_callback(self, msg):
        self.mode = msg.data
        self.get_logger().info(f"Setting mode to {msg.data}")

    # --------------------
    # CORE AUTONOMY API
    # --------------------
    def drive(self, speed_mps, steer_rad, duration):
        """
        Drive forward with Ackermann geometry for a fixed time.
        speed_mps:     Center linear velocity (m/s)
        steer_rad:     Steering angle at vehicle center (radians)
        duration:      Seconds
        """
        if self.mode != "AUTO":
            self.get_logger().warn("Not in AUTO mode → ignoring drive command.")
            return
        self.get_logger().info("In auto, going")
        steer_rad = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steer_rad))

        # Compute wheel speeds + angles (same logic as teleop)
        if abs(steer_rad) > 1e-6:
            R_turn = WHEELBASE / math.tan(abs(steer_rad))

            left_angle_abs = math.atan(WHEELBASE / (R_turn - TRACK_WIDTH / 2))
            right_angle_abs = math.atan(WHEELBASE / (R_turn + TRACK_WIDTH / 2))

            v_left = speed_mps * (R_turn - TRACK_WIDTH / 2) / R_turn
            v_right = speed_mps * (R_turn + TRACK_WIDTH / 2) / R_turn

            if steer_rad > 0:  # turning left
                fl_angle = left_angle_abs
                fr_angle = right_angle_abs
                fl_speed = v_left
                fr_speed = v_right
            else:             # turning right
                fl_angle = right_angle_abs
                fr_angle = left_angle_abs
                fl_angle = -fl_angle
                fr_angle = -fr_angle
                fl_speed = v_right
                fr_speed = v_left

        else:
            fl_angle = 0.0
            fr_angle = 0.0
            fl_speed = speed_mps
            fr_speed = speed_mps

        # Convert to wheel angular velocity (rad/s)
        fl_speed /= WHEEL_RADIUS
        fr_speed /= WHEEL_RADIUS

        # Publish for duration
        start = time.time()
        while time.time() - start < duration and rclpy.ok():
            if self.mode != "AUTO":
                self.stop()
                self.get_logger().warn("Left AUTO mode mid-drive → stopping.")
                return

            cmd_msg = Float64MultiArray()
            cmd_msg.data = [fl_speed, fr_speed, fl_angle, fr_angle]
            self.commanded_pub.publish(cmd_msg)

            time.sleep(0.05)  # 20 Hz publish rate

        self.stop()

    def stop(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0]
        self.commanded_pub.publish(msg)

    # --------------------
    # SAMPLE AUTONOMY (for testing)
    # --------------------
    def run_demo(self):
        """
        Example: straight 1 m/s for 2 sec, then left turn for 2 sec.
        """
        self.get_logger().info("Running autonomy test…")
        self.drive(0.75, 0.0, 20.0)       # Straight
        self.drive(1.0, 0.25, 20.0)      # Turn left


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNode()

    # Example usage (ensure ModeManager is set to AUTO!)
    node.run_demo()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
