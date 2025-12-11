import math
# Robot dimensions
WHEELBASE = 0.5207      # meters, distance front to rear axles
TRACK_WIDTH = 0.4953    # meters, distance between left and right front wheels
WHEEL_RADIUS = 0.0762   # meters, front wheel radius

# Joystick / vehicle limits
MAX_DRIVE_PRM_NO_LOAD = 3850 # rad/s
MAX_DRIVE_RPM_LOADED = 3000 # rad/s
DRIVE_MOTOR_GEAR_RATIO = 16/9.0 # gear ratio 16:9

# MAX_STEERING_ANGLE = 0.5235987756  # radians, ±30°
MAX_STEERING_ANGLE = 0.436332 # rad, 25 deg
# TODO: REAL MEASURED FULL POWER: 1.5 mph = 0.67 m/s
# Theoretical max speed 42.56 m/s
# THEORETICAL_MAX_SPEED = 42.56 # m/s actual measure # MAX_DRIVE_RPM_LOADED * DRIVE_MOTOR_GEAR_RATIO * WHEEL_RADIUS * 2 * 3.14159 / 60 # m/s, max forward speed with loads
# MAX SPEED constraints the car's linear speed, when steering, it can go up to 38 m/s currently to maintain this linear speed
MAX_SPEED_LINEAR = 0.5/2 # m/s limited to max linear velocity of the car
# Actual motor's max speed at MAX_SPEED_LINEAR, including when tunring at MAX_STEERING_ANGLE, currently 38 m/s
MAX_SPEED = 0.67 # MAX_SPEED_LINEAR * (1 + (TRACK_WIDTH * math.tan(abs(MAX_STEERING_ANGLE))) / (2 * WHEELBASE))

ENCODER_TICKS_PER_REV = 1200
