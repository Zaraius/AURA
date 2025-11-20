import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, String

import math
import time

from aura.constants import WHEELBASE, TRACK_WIDTH, WHEEL_RADIUS, MAX_STEERING_ANGLE


class TeleopAutoController(Node):
    def __init__(self):
        super().__init__("teleop_auto_controller")

        # Current mode: MANUAL, AUTO, STOP
        self.mode = "STOP"

        # Last command from joystick
        self.joy_msg = None

        # Publisher to the motor controller
        self.cmd_pub = self.create_publisher(Float64MultiArray, "/commanded", 10)

        # Subscribe to joystick
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        # Autonomous internal state
        self.auto_step = 0
        self.auto_start_time = None

        self.get_logger().info("Teleop + Auto Controller Initialized.")


    # ----------------------------------------
    # JOYSTICK CALLBACK
    # ----------------------------------------
    def joy_callback(self, msg: Joy):
        self.joy_msg = msg

        # Mode switching via buttons
        # A = MANUAL
        # B = AUTO
        # nothing pressed = STOP
        if msg.buttons[5] == 1:      # A
            self.set_mode("MANUAL")
            # self.get_logger().info("MANUALLLLL")
        elif msg.buttons[4] == 1:    # B
            self.set_mode("AUTO")
            # self.get_logger().info("AUTOOO")
        else:
            self.set_mode("STOP")
            # self.get_logger().info("STOPPPPP")


    def set_mode(self, new_mode):
        if new_mode != self.mode:
            self.mode = new_mode
            self.get_logger().info(f"Mode switched → {new_mode}")

            # When switching *out* of auto, reset autonomous sequence
            if new_mode != "AUTO":
                self.auto_step = 0
                self.auto_start_time = None


    # ----------------------------------------
    # MANUAL DRIVING (JOYSTICK)
    # ----------------------------------------
    def manual_drive(self):
        if not self.joy_msg:
            return None

        # Typical joystick mapping:
        throttle_axis = -self.joy_msg.axes[1]  # forward/back
        steer_axis = self.joy_msg.axes[2]      # left/right

        speed = throttle_axis * -1.0  # m/s
        steer = steer_axis * MAX_STEERING_ANGLE

        return self.ackermann_to_wheels(speed, steer)


    # ----------------------------------------
    # SIMPLE AUTONOMOUS SCRIPT (EDIT THIS)
    # ----------------------------------------
    def autonomous_drive(self):
        """
        Simple 3-step sequence:
        1. Drive straight 2s
        2. Turn left 2s
        3. Stop
        """
        now = time.time()
        if self.auto_start_time is None:
            self.auto_start_time = now

        elapsed = now - self.auto_start_time

        if self.auto_step == 0:
            # Step 1: straight
            if elapsed < 2.0:
                return self.ackermann_to_wheels(0.8, 0.0)
            else:
                self.auto_step += 1
                self.auto_start_time = now

        if self.auto_step == 1:
            # Step 2: gentle left turn
            if elapsed < 2.0:
                return self.ackermann_to_wheels(0.8, 0.25)
            else:
                self.auto_step += 1
                self.auto_start_time = now

        # Step 3: stop
        return [0.0, 0.0, 0.0, 0.0]


    # ----------------------------------------
    # ACKERMANN → WHEEL SPEEDS
    # ----------------------------------------
    def ackermann_to_wheels(self, speed_mps, steer_rad):
        steer_rad = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steer_rad))

        if abs(steer_rad) > 1e-6:
            R = WHEELBASE / math.tan(abs(steer_rad))
            left_abs = math.atan(WHEELBASE / (R - TRACK_WIDTH/2))
            right_abs = math.atan(WHEELBASE / (R + TRACK_WIDTH/2))

            # Wheel speeds
            v_left = speed_mps * (R - TRACK_WIDTH/2) / R
            v_right = speed_mps * (R + TRACK_WIDTH/2) / R

            # Assign inside/outside
            if steer_rad > 0:
                fl_angle = left_abs
                fr_angle = right_abs
                fl_speed = v_left
                fr_speed = v_right
            else:
                fl_angle = -right_abs
                fr_angle = -left_abs
                fl_speed = v_right
                fr_speed = v_left
        else:
            fl_angle = fr_angle = 0.0
            fl_speed = fr_speed = speed_mps

        # Convert to rad/s
        fl_speed /= WHEEL_RADIUS
        fr_speed /= WHEEL_RADIUS

        max_allowed = speed_mps / WHEEL_RADIUS  # or from joystick
        max_raw = max(abs(fl_speed), abs(fr_speed))

        if max_raw > max_allowed:
            scale = max_allowed / max_raw
            fl_speed *= scale
            fr_speed *= scale
            
        return [fl_speed, fr_speed, fl_angle, fr_angle]


    # ----------------------------------------
    # MAIN UPDATE LOOP (50 Hz)
    # ----------------------------------------
    def update(self):
        msg = Float64MultiArray()

        if self.mode == "MANUAL":
            cmd = self.manual_drive()
            if cmd:
                msg.data = cmd
                self.cmd_pub.publish(msg)
            return

        if self.mode == "AUTO":
            cmd = self.autonomous_drive()
            msg.data = cmd
            self.cmd_pub.publish(msg)
            return

        # STOP mode → publish zero only once
        msg.data = [0.0, 0.0, 0.0, 0.0]
        self.cmd_pub.publish(msg)


# ----------------------------------------
# MAIN
# ----------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TeleopAutoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
