import RPi.GPIO as GPIO
import pigpio # type: ignore
import time
import math

# === Stepper Motor Setup ===
STEP_PIN = 21
DIR_PIN = 20
LIMIT_SWITCH_PIN = 6
STEPS_PER_REV = 200  # 200 steps for one full revolution
MAX_ANGLE = 100  # Maximum allowed movement in degrees from the current position
current_position = 0  # Tracks current position in degrees

# Setup GPIO for stepper motor
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# === Servo Motor Setup ===
pi = pigpio.pi()
servo_pins = [22, 23, 24, 27]  # GPIO pins for the 4 servos, adding pin 27 for the fingers
servo_current_angles = [0, 0, 0, 0]  # Store the current angle of each servo

# === Robotic Arm Link Lengths (Adjust according to your arm's dimensions) ===
L1 = 51  # Base to shoulder
L2 = 140  # Shoulder to elbow
L3 = 140  # Elbow to wrist

# Function to convert degrees to radians
def deg_to_rad(deg):
    return deg * (math.pi / 180.0)

# Forward Kinematics function to calculate end-effector position
def forward_kinematics(theta1, theta2, theta3, theta4):
    theta1_rad = deg_to_rad(theta1)
    theta2_rad = deg_to_rad(theta2)
    theta3_rad = deg_to_rad(theta3)
    theta4_rad = deg_to_rad(theta4)

    shoulder_x = L1 * math.cos(theta1_rad)
    shoulder_y = L1 * math.sin(theta1_rad)
    shoulder_z = L1  # Base height assumed along z-axis

    elbow_x = shoulder_x + L2 * math.cos(theta2_rad) * math.cos(theta1_rad)
    elbow_y = shoulder_y + L2 * math.cos(theta2_rad) * math.sin(theta1_rad)
    elbow_z = shoulder_z + L2 * math.sin(theta2_rad)

    wrist_x = elbow_x + L3 * math.cos(theta3_rad) * math.cos(theta1_rad)
    wrist_y = elbow_y + L3 * math.cos(theta3_rad) * math.sin(theta1_rad)
    wrist_z = elbow_z + L3 * math.sin(theta3_rad)

    return wrist_x, wrist_y, wrist_z

# Function to slowly set the servo angle with smooth movement
def set_servo_angle_slow(pin, start_angle, end_angle, step_delay=0.05):
    if start_angle < end_angle:
        step = 1  # Increment angle
    else:
        step = -1  # Decrement angle

    for angle in range(int(start_angle), int(end_angle), step):
        pulse_width = int(500 + (angle / 180.0) * 2000)
        pi.set_servo_pulsewidth(pin, pulse_width)
        time.sleep(step_delay)

    pulse_width = int(500 + (end_angle / 180.0) * 2000)
    pi.set_servo_pulsewidth(pin, pulse_width)

# Stepper Motor Calibration
def calibrate_stepper():
    global current_position, servo_current_angles  # Declare both global variables
    print("Calibrating the stepper motor...")

    GPIO.output(DIR_PIN, GPIO.LOW)

    while GPIO.input(LIMIT_SWITCH_PIN) == GPIO.HIGH:
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.01)

    current_position = 0
    print("Calibration complete. 0-degree position set.")

    # Move servos to home position after calibration
    for i in range(len(servo_pins)):
        set_servo_angle_slow(servo_pins[i], servo_current_angles[i], 0)  # Move each servo to 0 degrees
        servo_current_angles[i] = 0  # Update current angles to 0

# Move Stepper Motor to a Target Angle
def move_stepper(target_angle):
    global current_position
    min_allowed_angle = current_position - MAX_ANGLE
    max_allowed_angle = current_position + MAX_ANGLE

    if target_angle < min_allowed_angle:
        print(f"Target angle is below the allowable limit. Moving to {min_allowed_angle} degrees instead.")
        target_angle = min_allowed_angle
    elif target_angle > max_allowed_angle:
        print(f"Target angle is above the allowable limit. Moving to {max_allowed_angle} degrees instead.")
        target_angle = max_allowed_angle

    # Calculate the angle to move
    angle_to_move = target_angle - current_position
    if angle_to_move == 0:
        print("No movement required.")
        return

    steps = int((STEPS_PER_REV * abs(angle_to_move)) / 360)

    if angle_to_move > 0:
        GPIO.output(DIR_PIN, GPIO.HIGH)  # Move forward
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)   # Move backward

    # Step the motor the required number of steps
    for i in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.01)

    current_position = target_angle
    print(f"Stepper moved to: {current_position} degrees")

# === Main Program ===
try:
    home_position = [0, 150, 180, 0, 0]  # Base, Shoulder, Elbow, Wrist, Fingers angles in degrees
    drop_position = [90, 70, 120, 70, 50]  # Example position including fingers angle

    # Step 1: Calibrate the stepper motor
    calibrate_stepper()

    # After calibration, move servos to home position
    print("Moving servos to home position...")
    for i in range(len(servo_pins)):
        set_servo_angle_slow(servo_pins[i], servo_current_angles[i], home_position[i + 1])  # Move to home position
        servo_current_angles[i] = home_position[i + 1]  # Update current angles

    # Step 3: Get user input for the reach position
    print("Please enter the reach position angles:")
    base_angle = float(input("Base (stepper) angle: "))
    shoulder_angle = float(input("Shoulder (servo 1) angle: "))
    elbow_angle = float(input("Elbow (servo 2) angle: "))
    wrist_angle = float(input("Wrist (servo 3) angle: "))
    fingers_angle = float(input("Fingers (servo 4) angle: "))
    reach_position = [base_angle, shoulder_angle, elbow_angle, wrist_angle, fingers_angle]

    # Step 6: Move to user-input reach position
    print("Moving to Reach position...")
    move_stepper(reach_position[0])  # Base (stepper motor)
    set_servo_angle_slow(servo_pins[0], servo_current_angles[0], reach_position[1])  # Shoulder
    set_servo_angle_slow(servo_pins[1], servo_current_angles[1], reach_position[2])  # Elbow
    set_servo_angle_slow(servo_pins[2], servo_current_angles[2], reach_position[3])  # Wrist
    set_servo_angle_slow(servo_pins[3], servo_current_angles[3], reach_position[4])  # Fingers

    # Update current servo angles
    servo_current_angles = reach_position[1:]

    # Add a delay of 3 seconds before moving to drop position
    print("Waiting for 3 seconds before moving to drop position...")
    time.sleep(3)

    # Step 7: Move to drop position
    print("Moving to Drop position...")
    move_stepper(drop_position[0])  # Base (stepper motor)
    set_servo_angle_slow(servo_pins[0], servo_current_angles[0], drop_position[1])  # Shoulder
    set_servo_angle_slow(servo_pins[1], servo_current_angles[1], drop_position[2])  # Elbow
    set_servo_angle_slow(servo_pins[2], servo_current_angles[2], drop_position[3])  # Wrist
    set_servo_angle_slow(servo_pins[3], servo_current_angles[3], drop_position[4])  # Fingers

    # Update current servo angles after reaching drop position
    servo_current_angles = drop_position[1:]

    # Move back to home position
    print("Returning to home position...")
    move_stepper(0)  # Move stepper back to home position
    for i in range(len(servo_pins)):
        set_servo_angle_slow(servo_pins[i], servo_current_angles[i], home_position[i + 1])  # Move to home position
        servo_current_angles[i] = home_position[i + 1]  # Update current angles

    print("Movement complete!")

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    # Cleanup GPIO and servos on exit
    GPIO.cleanup()  # Clean up stepper
