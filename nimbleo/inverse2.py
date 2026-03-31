import RPi.GPIO as GPIO
import pigpio
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
servo_pins = [22, 23, 24, 27]  # GPIO pins for the 4 servos
servo_current_angles = [120, 0, 0, 0]  # Store the current angle of each servo

# Robotic arm link lengths (in mm)
L1 = 51.25
L2 = 139.19
L3 = 139.19
L4 = 74.83

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
    global current_position, servo_current_angles
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

# Inverse Kinematics Function to calculate θ1, θ2, θ3
def inverse_kinematics(x, y, z):
    # Base angle θ1
    theta_1 = math.atan2(y, x) * (180 / math.pi)  # Convert to degrees

    # Calculate distance to wrist position (ignoring L4 for now)
    d_wrist = math.sqrt(x**2 + y**2) - L4  # Effective distance on XY plane
    z_wrist = z - L1  # Adjust Z to account for L1 (base height)

    # Calculate angles θ2 and θ3 using the law of cosines
    D = (d_wrist**2 + z_wrist**2 - L2**2 - L3**2) / (2 * L2 * L3)
    theta_3 = math.acos(D) * (180 / math.pi)  # Elbow angle in degrees

    # Shoulder angle θ2
    theta_2 = math.atan2(z_wrist, d_wrist) - math.atan2(L3 * math.sin(math.radians(theta_3)), L2 + L3 * math.cos(math.radians(theta_3)))
    theta_2 = theta_2 * (180 / math.pi)  # Convert to degrees

    # Ensure that theta_2 is positive (magnitude)
    theta_2 = abs(theta_2)  # Make θ2 magnitude always positive

    print(f"Inverse Kinematics - Base (θ1): {theta_1:.2f}, Shoulder (θ2): {theta_2:.2f}, Elbow (θ3): {theta_3:.2f}")
    
    return theta_1, theta_2, theta_3

# === Main Program ===
try:
    home_position = [0, 150, 180, 0, 0]  # Base, Shoulder, Elbow, Wrist, Fingers angles in degrees
    drop_position = [90, 70, 120, 70, ]  # Example position including fingers angle

    # Step 1: Calibrate the stepper motor
    calibrate_stepper()

    # After calibration, move servos to home position
    print("Moving servos to home position...")
    for i in range(len(servo_pins)):
        set_servo_angle_slow(servo_pins[i], servo_current_angles[i], home_position[i + 1])  # Move to home position
        servo_current_angles[i] = home_position[i + 1]  # Update current angles

    # Step 3: Get user input for the end effector position (x, y, z)
    print("Please enter the reach position coordinates:")
    x = float(input("Enter x-coordinate: "))
    y = float(input("Enter y-coordinate: "))
    z = float(input("Enter z-coordinate: "))

    # Calculate the joint angles using inverse kinematics
    theta_1, theta_2, theta_3 = inverse_kinematics(x, y, z)

    # Step 4: Move to the calculated position in the order: Base, Finger, Wrist, Elbow, Shoulder
    print("Moving to Reach position...")

    # Base (stepper motor)
    move_stepper(theta_1)
    
    # Fingers (assuming last servo)
    set_servo_angle_slow(servo_pins[3], servo_current_angles[3], 90)  # Example finger position
    servo_current_angles[3] = 90  # Update fingers angle

    # Wrist (servo 2)
    set_servo_angle_slow(servo_pins[2], servo_current_angles[2], 70)  # Example wrist position
    servo_current_angles[2] = 70  # Update wrist angle

    # Shoulder (servo 0)
    set_servo_angle_slow(servo_pins[0], servo_current_angles[0], theta_2)  # Move shoulder
    servo_current_angles[0] = theta_2  # Update shoulder angle

    # Elbow (servo 1)
    set_servo_angle_slow(servo_pins[1], servo_current_angles[1], theta_3)  # Move elbow
    servo_current_angles[1] = theta_3  # Update elbow angle


    # Add a delay before moving to drop position
    print("Waiting for 3 seconds before moving to drop position...")
    time.sleep(3)

    # Step 5: Move to drop position (for example)
    print("Moving to Drop position...")
    move_stepper(drop_position[0])  # Base (stepper motor)
    set_servo_angle_slow(servo_pins[0], servo_current_angles[0], drop_position[1])  # Shoulder
    set_servo_angle_slow(servo_pins[1], servo_current_angles[1], drop_position[2])  # Elbow
    set_servo_angle_slow(servo_pins[2], servo_current_angles[2], drop_position[3])  # Wrist
    set_servo_angle_slow(servo_pins[3], servo_current_angles[3], drop_position[4])  # Fingers

    # Return to Home Position
    print("Returning to Home position...")
    move_stepper(home_position[0])  # Base
    for i in range(len(servo_pins)):
        set_servo_angle_slow(servo_pins[i], servo_current_angles[i], home_position[i + 1])  # Move to home position
        servo_current_angles[i] = home_position[i + 1]  # Update current angles

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    GPIO.cleanup()
    pi.stop()
