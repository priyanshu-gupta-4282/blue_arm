import RPi.GPIO as GPIO
import pigpio
import time

# === Stepper Motor Setup ===
STEP_PIN = 21
DIR_PIN = 20
LIMIT_SWITCH_PIN = 6
STEPS_PER_REV = 200  # 200 steps for one full revolution
MAX_ANGLE = 100  # Maximum allowed movement in degrees from 0 to 100
current_position = 0  # Tracks current position in degrees

# Setup GPIO for stepper motor
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# === Servo Motor Setup ===
pi = pigpio.pi()
servo_pins = [22, 23, 24]  # GPIO pins for the 3 servos

# Function to slowly set the servo angle with smooth movement
def set_servo_angle_slow(pin, start_angle, end_angle, step_delay=0.05):
    # Determine the direction of movement
    if start_angle < end_angle:
        step = 1  # Incrementing the angle
    else:
        step = -1  # Decrementing the angle
    
    # Slowly move the servo in small steps
    for angle in range(int(start_angle), int(end_angle), step):
        pulse_width = int(500 + (angle / 180.0) * 2000)
        pi.set_servo_pulsewidth(pin, pulse_width)
        time.sleep(step_delay)  # Adjust delay for speed control

    # Ensure it reaches the final position
    pulse_width = int(500 + (end_angle / 180.0) * 2000)
    pi.set_servo_pulsewidth(pin, pulse_width)

# === Stepper Motor Functions ===
def calibrate_stepper():
    global current_position
    print("Calibrating the stepper motor...")

    # Move counterclockwise at a slower speed until the limit switch is triggered
    GPIO.output(DIR_PIN, GPIO.LOW)

    while GPIO.input(LIMIT_SWITCH_PIN) == GPIO.HIGH:
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.01)

    # Limit switch hit, this is 0-degree position
    current_position = 0
    print("Calibration complete. 0-degree position set.")

def move_stepper(angle):
    global current_position
    target_position = current_position + angle

    if target_position < 0:
        print("Error: Cannot move below 0 degrees.")
        return

    if target_position > MAX_ANGLE:
        print(f"Error: Cannot move beyond {MAX_ANGLE} degrees.")
        return

    steps = int((STEPS_PER_REV * abs(angle)) / 360)

    if angle > 0:
        GPIO.output(DIR_PIN, GPIO.HIGH)
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)

    for i in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.01)

    current_position = target_position
    print(f"Stepper moved to: {current_position} degrees")

# === Main Program ===
try:
    # Step 1: Calibrate the stepper motor
    calibrate_stepper()

    # Step 2: Get angles from the user
    base_angle = float(input("Enter the base (stepper) angle: "))
    shoulder_angle = float(input("Enter the shoulder (servo 1) angle: "))
    elbow_angle = float(input("Enter the elbow (servo 2) angle: "))
    wrist_angle = float(input("Enter the wrist (servo 3) angle: "))

    # Track the current positions of the servos (assumed starting at 90 degrees)
    current_shoulder_angle = 90
    current_elbow_angle = 90
    current_wrist_angle = 90

    # Step 3: Move everything at once
    print("Moving the robotic arm...")

    # Move the stepper for the base
    move_stepper(base_angle)

    # Smoothly move servos for shoulder, elbow, and wrist
    set_servo_angle_slow(servo_pins[0], current_shoulder_angle, shoulder_angle)
    set_servo_angle_slow(servo_pins[1], current_elbow_angle, elbow_angle)
    set_servo_angle_slow(servo_pins[2], current_wrist_angle, wrist_angle)
    time.sleep(4)

    print("Movement complete!")

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    # Cleanup GPIO and servos on exit
    GPIO.cleanup()  # Clean up stepper motor GPIO
    for pin in servo_pins:
        pi.set_servo_pulsewidth(pin, 0)  # Turn off the PWM signal for servos
    pi.stop()  # Stop pigpio library
    print("Stepper motor and servos turned off, program exited.")
