import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import RPi.GPIO as GPIO

# BCM pin definitions
IN1 = 11  # DIR left
IN2 = 13  # DIR right
AN1 = 12  # PWM left
AN2 = 33  # PWM right

class MODE:
    PWM_DIR = 0
    PWM_PWM = 1

class CytronMD:
    """Faithful Python port of Cytron's Arduino CytronMD library."""
    
    def __init__(self, mode, pin_pwm, pin_dir, frequency=20000):
        self._mode = mode
        self._pin_pwm = pin_pwm
        self._pin_dir = pin_dir
        
        GPIO.setup(self._pin_pwm, GPIO.OUT)
        GPIO.setup(self._pin_dir, GPIO.OUT)
        GPIO.output(self._pin_pwm, GPIO.LOW)
        GPIO.output(self._pin_dir, GPIO.LOW)
        
        self.pwm = GPIO.PWM(self._pin_pwm, frequency)
        self.pwm.start(0)
    
    def setSpeed(self, speed):
        """Matches the Arduino library behavior exactly."""
        if speed > 255:
            speed = 255
        elif speed < -255:
            speed = -255
        
        # MATCHES ARDUINO VERSION EXACTLY
        if self._mode == MODE.PWM_DIR:
            if speed > 0:
                duty = speed / 255 * 100
                print(f"duty: {duty}")
                self.pwm.ChangeDutyCycle(duty)
                GPIO.output(self._pin_dir, GPIO.LOW)  # forward
                print("FORWARDDDDD")
            elif speed < 0:
                duty = -speed / 255 * 100
                print(f"duty: {duty}")
                self.pwm.ChangeDutyCycle(duty)
                GPIO.output(self._pin_dir, GPIO.HIGH)  # reverse
                print("REVERSSEEEE")
        elif self._mode == MODE.PWM_PWM:
            # (not used on Raspberry Pi)
            pass
    
    def cleanup(self):
        self.pwm.stop()

class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # EXACT MATCH to Arduino library
        self.motor_left = CytronMD(MODE.PWM_DIR, AN1, IN1)
        self.motor_right = CytronMD(MODE.PWM_DIR, AN2, IN2)
        
        self.drive_sub = self.create_subscription(
            Float64MultiArray, "/commanded", self.drive_callback, 10
        )
        
        self.get_logger().info("Motor controller initialized")
    
    def drive_callback(self, msg):
        if len(msg.data) != 4:
            self.get_logger().error("Expected 4 floats: [thrL, thrR, fl, fr]")
            return
        
        throttle_left = float(msg.data[0])
        throttle_right = float(msg.data[1])
        fl_angle = float(msg.data[2])
        fr_angle = float(msg.data[3])
        
        self.get_logger().info(
            f"L={throttle_left:.1f}, R={throttle_right:.1f}, "
            f"FL={fl_angle:.2f}, FR={fr_angle:.2f}"
        )
        
        # Send to hardware
        self.motor_left.setSpeed(throttle_left)
        self.motor_right.setSpeed(throttle_right)
    
    def cleanup(self):
        self.motor_left.setSpeed(0)
        self.motor_right.setSpeed(0)
        self.motor_left.cleanup()
        self.motor_right.cleanup()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
