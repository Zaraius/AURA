import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO
from std_msgs.msg import Int32

A_PIN = 17 # check with Jack and Zaraius what pins are being used
B_PIN = 27

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')

        self.position = 0
        self.publisher = self.create_publisher(Int32, 'encoder_position', 10)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        GPIO.setup(B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

        GPIO.add_event_detect(A_PIN, GPIO.BOTH, callback=self.handle_A)

        # Timer just publishes the latest position at a steady rate
        self.timer = self.create_timer(0.02, self.publish_position)  # 50 Hz

    def handle_A(self, channel):
        a_state = GPIO.input(A_PIN)
        b_state = GPIO.input(B_PIN)

        # Quadrature direction logic
        if a_state == b_state:
            self.position += 1
        else:
            self.position -= 1

    def publish_position(self):
        msg = Int32()
        msg.data = self.position
        self.publisher.publish(msg)

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
