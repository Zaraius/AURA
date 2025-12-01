'''
Code by Rodrigo C. Curiel, 2024. 
Check this and more snippets at www.curielrodrigo.com 
'''

# Import the required libraries
import RPi.GPIO as GPIO
from time import sleep

# Setup the GPIO pins (could be different for your particular case)
Enc_A = 2
Enc_B = 3

# Position counter
counter = 0

# Encoder state
last_A = 0
last_B = 0

# Make the initial setup for the assigned pins on the RPI board
def init():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Enc_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(Enc_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Define a sample reading function for the encoder phases
def read_encoder():
    return GPIO.input(Enc_A), GPIO.input(Enc_B)

# Define a simple updating function for the counter 
def update_position(A, B):
    global counter, last_A, last_B
    if A != last_A or B != last_B:  # Only update on change
        if A == 1 and B == 0 and last_A == 0:  # Clockwise
            counter += 1
        elif A == 0 and B == 1 and last_B == 0:  # Counter-clockwise
            counter -= 1
        last_A, last_B = A, B

# Our main loop function 
def main():
    init()
    print("Monitoring rotary encoder...")
    try:
        while True:
            A, B = read_encoder()
            update_position(A, B)
            print(f"Position: {counter}")
            # This sleep should be short to avoid loosing steps on your reading. It might require some tunning for your use case
            sleep(.0000001)
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Program exited cleanly")

# When this script is called, run the main function
if __name__ == '__main__':
    main()