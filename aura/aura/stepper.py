import RPi.GPIO as GPIO
from time import sleep

# Direction pin from controller
DIR = 10
# Step pin from controller
STEP = 8
# 0/1 used to signify clockwise or counterclockwise.
CW = 1
CCW = 0
STEPS = 200         # 200 steps for 360 degree
SLEEP_TIME = 0.0005 # delay between step pin toggles

# Setup pin layout on PI
GPIO.setmode(GPIO.BOARD)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

# Set the initial direction
GPIO.output(DIR, CW)
print("running")
try:
  while True:
    # Change direction: changing direction requires time to switch.
    # The time is dictated by the stepper motor and controller.
    sleep(1.0)

    # Clockwise
    GPIO.output(DIR, CW)
    for _ in range(STEPS):
      GPIO.output(STEP, GPIO.HIGH)
      sleep(SLEEP_TIME)
      GPIO.output(STEP, GPIO.LOW)
      sleep(SLEEP_TIME)

    sleep(1.0)

    # Counter-clockwise
    GPIO.output(DIR, CCW)
    for _ in range(STEPS):
      GPIO.output(STEP, GPIO.HIGH)
      sleep(SLEEP_TIME)
      GPIO.output(STEP, GPIO.LOW)
      sleep(SLEEP_TIME)

except KeyboardInterrupt:
  print("cleanup")
finally:
  GPIO.cleanup()
