#!/usr/bin/env python3
"""
Cytron Motor Driver Test Script for Raspberry Pi
Equivalent to the Arduino example code
"""

import time
from cytron_motor_driver import CytronMD, MODE

# Pin definitions (BCM numbering)
IN1 = 4   # Direction pin for left motor
IN2 = 10  # Direction pin for right motor
AN1 = 3   # PWM pin for left motor (GPIO3/SCL)
AN2 = 9   # PWM pin for right motor (GPIO9/MISO)

# Note: On Raspberry Pi, these GPIO pins support hardware PWM:
# GPIO12, GPIO13, GPIO18, GPIO19
# For best performance, consider using these pins instead:
# AN1 = 18  # PWM pin for left motor
# AN2 = 19  # PWM pin for right motor

def main():
    print("Setting up motors...")
    
    # Initialize motor drivers with PWM_DIR mode
    motor1 = CytronMD(MODE.PWM_DIR, AN1, IN1, pwm_freq=1000)
    motor2 = CytronMD(MODE.PWM_DIR, AN2, IN2, pwm_freq=1000)
    
    print("Setup done")
    
    try:
        while True:
            # Motor 1 forward at 50%, Motor 2 backward at 50%
            print("Motors running at 50% speed")
            motor1.setSpeed(128)
            motor2.setSpeed(-128)
            time.sleep(1)
            
            # Stop both motors
            print("Motors stopped")
            motor1.setSpeed(0)
            motor2.setSpeed(0)
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nStopping motors and cleaning up...")
        motor1.stop()
        motor2.stop()
        motor1.cleanup()
        motor2.cleanup()
        print("Cleanup complete")

if __name__ == "__main__":
    main()