# Robot dimensions
WHEELBASE = 0.5207      # meters, distance front to rear axles
TRACK_WIDTH = 0.4953    # meters, distance between left and right front wheels
WHEEL_RADIUS = 0.0762   # meters, front wheel radius

# Joystick / vehicle limits
MAX_DRIVE_PRM_NO_LOAD = 3850 # rad/s
MAX_DRIVE_RPM_LOADED = 3000 # rad/s
DRIVE_MOTOR_GEAR_RATIO = 16/9 # gear ratio 16:9

# Theoretical max speed 13.47 m/s
MOTOR_MAX_SPEED = 13.47 # MAX_DRIVE_RPM_LOADED / DRIVE_MOTOR_GEAR_RATIO * WHEEL_RADIUS * 2 * 3.14159 / 60 # m/s, max forward speed with loads
MAX_SPEED = 2.2352 # Limited to 5 mph software max speed
MAX_STEERING_ANGLE = 0.5235987756  # radians, ±30°
