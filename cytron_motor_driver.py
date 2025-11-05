"""
Cytron Motor Driver Library for Raspberry Pi
Supports PWM_DIR and PWM_PWM modes
"""

import RPi.GPIO as GPIO
from enum import Enum

class MODE(Enum):
    PWM_DIR = 1
    PWM_PWM = 2

class CytronMD:
    def __init__(self, mode, pin1, pin2, pwm_freq=1000):
        """
        Initialize Cytron Motor Driver
        
        Args:
            mode: MODE.PWM_DIR or MODE.PWM_PWM
            pin1: PWM pin (BCM numbering)
            pin2: DIR pin for PWM_DIR mode, or second PWM pin for PWM_PWM mode
            pwm_freq: PWM frequency in Hz (default 1000)
        """
        self._mode = mode
        self._pin1 = pin1
        self._pin2 = pin2
        self._pwm_freq = pwm_freq
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Configure pins
        GPIO.setup(self._pin1, GPIO.OUT)
        GPIO.setup(self._pin2, GPIO.OUT)
        
        # Initialize PWM on pin1
        self._pwm1 = GPIO.PWM(self._pin1, self._pwm_freq)
        self._pwm1.start(0)
        
        # Initialize PWM on pin2 if PWM_PWM mode
        if self._mode == MODE.PWM_PWM:
            self._pwm2 = GPIO.PWM(self._pin2, self._pwm_freq)
            self._pwm2.start(0)
        else:
            self._pwm2 = None
            GPIO.output(self._pin2, GPIO.LOW)
    
    def setSpeed(self, speed):
        """
        Set motor speed and direction
        
        Args:
            speed: -255 to 255 (negative = reverse, positive = forward)
        """
        # Clamp speed to valid range
        if speed > 255:
            speed = 255
        elif speed < -255:
            speed = -255
        
        # Convert to duty cycle (0-100%)
        duty_cycle = abs(speed) * 100.0 / 255.0
        
        # Set speed and direction based on mode
        if self._mode == MODE.PWM_DIR:
            if speed >= 0:
                self._pwm1.ChangeDutyCycle(duty_cycle)
                GPIO.output(self._pin2, GPIO.LOW)
            else:
                self._pwm1.ChangeDutyCycle(duty_cycle)
                GPIO.output(self._pin2, GPIO.HIGH)
        
        elif self._mode == MODE.PWM_PWM:
            if speed >= 0:
                self._pwm1.ChangeDutyCycle(duty_cycle)
                self._pwm2.ChangeDutyCycle(0)
            else:
                self._pwm1.ChangeDutyCycle(0)
                self._pwm2.ChangeDutyCycle(duty_cycle)
    
    def stop(self):
        """Stop the motor"""
        self.setSpeed(0)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self._pwm1.stop()
        if self._pwm2:
            self._pwm2.stop()
        GPIO.cleanup([self._pin1, self._pin2])