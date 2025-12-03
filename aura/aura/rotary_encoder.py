import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Int32

# Left encoder pins
A_PIN = 23
B_PIN = 24

# Right encoder pins
C_PIN = 25
D_PIN = 8


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')

        # Positions
        self.left_pos = 0
        self.right_pos = 0

        # Last states
        self.last_a = 0
        self.last_c = 0

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.setup(C_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(D_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Poll at 1 kHz
        self.poll_timer = self.create_timer(0.001, self.poll_encoders)

        # Publish 50 Hz
        self.pub_left = self.create_publisher(Int32, 'left_encoder', 10)
        self.pub_right = self.create_publisher(Int32, 'right_encoder', 10)

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

    def publish_positions(self):
        msg_left = Int32()
        msg_left.data = self.left_pos
        self.pub_left.publish(msg_left)

        msg_right = Int32()
        msg_right.data = self.right_pos
        self.pub_right.publish(msg_right)


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
