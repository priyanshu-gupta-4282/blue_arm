import RPi.GPIO as GPIO
import time

# === Stepper Motor Setup ===
STEP_PIN = 21
DIR_PIN = 20
LIMIT_SWITCH_PIN = 6
ENABLE_PIN = 19  # Enable pin for the stepper driver
STEPS_PER_REV = 200  # 200 steps for one full revolution
MAX_ANGLE = 180  # Maximum allowed movement in degrees from current position
current_position = 0  # Tracks current position in degrees
STEP_DELAY = 0.03  # Reduced delay for faster stepping

# Setup GPIO for stepper motor
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENABLE_PIN, GPIO.OUT)  # Set the enable pin as an output

# Enable the stepper motor driver
def enable_motor():
    GPIO.output(ENABLE_PIN, GPIO.LOW)  # Typically, LOW enables the motor

# Disable the stepper motor driver
def disable_motor():
    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Typically, HIGH disables the motor

# Stepper Motor Calibration
def calibrate_stepper():
    global current_position
    print("Calibrating the stepper motor...")
    enable_motor()  # Enable motor during calibration

    # Move the stepper in reverse until it hits the limit switch
    GPIO.output(DIR_PIN, GPIO.LOW)

    while GPIO.input(LIMIT_SWITCH_PIN) == GPIO.HIGH:
        step_motor()
        time.sleep(STEP_DELAY)

    current_position = 0
    print("Calibration complete. 0-degree position set.")
    disable_motor()  # Disable motor after calibration

# Helper function to step the motor once
def step_motor():
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(STEP_DELAY / 2)  # Pulse width adjustment for faster stepping
    GPIO.output(STEP_PIN, GPIO.LOW)
    time.sleep(STEP_DELAY / 2)

# Move Stepper Motor to a Target Angle
def move_to_angle(target_angle):
    global current_position

    # Calculate limits only if current_position changes
    min_allowed_angle = current_position - MAX_ANGLE
    max_allowed_angle = current_position + MAX_ANGLE

    if target_angle < min_allowed_angle:
        print(f"Target angle is below the allowable limit. Moving to {min_allowed_angle} degrees instead.")
        target_angle = min_allowed_angle
    elif target_angle > max_allowed_angle:
        print(f"Target angle is above the allowable limit. Moving to {max_allowed_angle} degrees instead.")
        target_angle = max_allowed_angle

    # Calculate the number of steps required to move to the target angle
    angle_difference = target_angle - current_position
    steps = int((STEPS_PER_REV * abs(angle_difference)) / 360)

    # Set the direction only if it changes
    if angle_difference > 0:
        GPIO.output(DIR_PIN, GPIO.HIGH)  # Move forward
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)  # Move backward

    enable_motor()  # Enable the motor before moving

    # Step the motor the required number of steps
    for i in range(steps):
        step_motor()

    # Update the current position
    current_position = target_angle
    print(f"Stepper moved to: {current_position} degrees")
    disable_motor()  # Disable the motor after movement

# === Main Program ===
try:
    # Step 1: Calibrate the stepper motor
    calibrate_stepper()

    # Step 2: Get user input for the target angle and move the stepper motor
    while True:
        target_angle = float(input(f"Enter a target angle (within ±{MAX_ANGLE} degrees from current position): "))
        move_to_angle(target_angle)

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    GPIO.cleanup()  # Clean up GPIO on exit
    print("Stepper motor control exited.")
