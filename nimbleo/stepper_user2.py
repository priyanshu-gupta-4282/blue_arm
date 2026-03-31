import RPi.GPIO as GPIO
import time

# Define GPIO pin numbers
STEP_PIN = 21
DIR_PIN = 20
LIMIT_SWITCH_PIN = 6
STEPS_PER_REV = 200  # 200 steps for one full revolution

MAX_ANGLE = 100  # Maximum allowed movement in degrees from 0 to 100
current_position = 0  # Tracks current position in degrees

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def calibrate_stepper():
    global current_position
    print("Calibrating...")

    # Move counterclockwise at a slower speed until the limit switch is triggered
    GPIO.output(DIR_PIN, GPIO.LOW)  # Set direction to counterclockwise

    while GPIO.input(LIMIT_SWITCH_PIN) == GPIO.HIGH:
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.01)  # Slow down calibration by increasing the delay
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.01)  # Adjust this delay to control speed

    # Limit switch hit, so this is the 0-degree position
    current_position = 0
    print("Calibration complete. 0-degree position set.")

def move_stepper(angle):
    global current_position
    target_position = current_position + angle

    # Prevent negative movement beyond 0 degrees
    if target_position < 0:
        print("Error: Cannot move below 0 degrees.")
        return

    # Prevent movement beyond the 100-degree limit
    if target_position > MAX_ANGLE:
        print(f"Error: Cannot move beyond {MAX_ANGLE} degrees.")
        return

    # Convert the angle to steps
    steps = int((STEPS_PER_REV * abs(angle)) / 360)

    # Set direction based on the angle
    if angle > 0:
        GPIO.output(DIR_PIN, GPIO.HIGH)  # Clockwise
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)  # Counterclockwise

    # Move the stepper motor
    for i in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.01)  # Small delay to create a pulse
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.01)  # Delay between steps

    # Update the current position
    current_position = target_position
    print(f"New position: {current_position} degrees")

try:
    # Calibrate the stepper motor on startup
    calibrate_stepper()

    while True:
        # Get angle input from the user
        angle_input = input("Enter the angle to rotate (positive for CW, negative for CCW): ")

        try:
            angle = float(angle_input)  # Convert input to a float
            move_stepper(angle)
        except ValueError:
            print("Please enter a valid number for the angle.")

except KeyboardInterrupt:
    print("Program stopped by user.")
finally:
    GPIO.cleanup()  # Clean up GPIO on exit
