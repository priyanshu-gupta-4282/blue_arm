import RPi.GPIO as GPIO
import pigpio
import time
import math

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
    global current_position
    print("Calibrating the stepper motor...")

    GPIO.output(DIR_PIN, GPIO.LOW)

    while GPIO.input(LIMIT_SWITCH_PIN) == GPIO.HIGH:
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.01)

    current_position = 0
    print("Calibration complete. 0-degree position set.")

# Move Stepper Motor to an Angle
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
     
    home_position = [0, 150, 180, 0]  # Base, Shoulder, Elbow, Wrist angles in degrees
    drop_position = [90, 70, 120, 70] 
    # Step 1: Calibrate the stepper motor
    calibrate_stepper()

    home_x, home_y, home_z = forward_kinematics(*home_position)
    print(f"End-effector position at Home: X={home_x}, Y={home_y}, Z={home_z}")

    # Step 5: Move to home position
    print("Moving to Home position...")
    move_stepper(home_position[0])  # Base (stepper motor)
    set_servo_angle_slow(servo_pins[0], 90, home_position[1])  # Shoulder
    set_servo_angle_slow(servo_pins[1], 90, home_position[2])  # Elbow
    set_servo_angle_slow(servo_pins[2], 90, home_position[3])  # Wrist
    time.sleep(5)

    # Step 2: Predefined home and drop positions
    home_position = [0, 150, 180, 0]  # Base, Shoulder, Elbow, Wrist angles in degrees
    drop_position = [90, 70, 120, 70]  # Base, Shoulder, Elbow, Wrist angles in degrees

    # Step 3: Get user input for the reach position
    print("Please enter the reach position angles:")
    base_angle = float(input("Base (stepper) angle: "))
    shoulder_angle = float(input("Shoulder (servo 1) angle: "))
    elbow_angle = float(input("Elbow (servo 2) angle: "))
    wrist_angle = float(input("Wrist (servo 3) angle: "))
    reach_position = [base_angle, shoulder_angle, elbow_angle, wrist_angle]

    # Step 4: Forward kinematics for each position
    home_x, home_y, home_z = forward_kinematics(*home_position)
    print(f"End-effector position at Home: X={home_x}, Y={home_y}, Z={home_z}")

    reach_x, reach_y, reach_z = forward_kinematics(*reach_position)
    print(f"End-effector position at Reach: X={reach_x}, Y={reach_y}, Z={reach_z}")

    drop_x, drop_y, drop_z = forward_kinematics(*drop_position)
    print(f"End-effector position at Drop: X={drop_x}, Y={drop_y}, Z={drop_z}")

    # Step 6: Move to user-input reach position
    print("Moving to Reach position...")
    move_stepper(reach_position[0])  # Base (stepper motor)
    set_servo_angle_slow(servo_pins[0], home_position[1], reach_position[1])  # Shoulder
    set_servo_angle_slow(servo_pins[1], home_position[2], reach_position[2])  # Elbow
    set_servo_angle_slow(servo_pins[2], home_position[3], reach_position[3])  # Wrist
    time.sleep(5)

    # Step 7: Move to drop position
    print("Moving to Drop position...")
    move_stepper(drop_position[0])  # Base (stepper motor)
    set_servo_angle_slow(servo_pins[0], reach_position[1], drop_position[1])  # Shoulder
    set_servo_angle_slow(servo_pins[1], reach_position[2], drop_position[2])  # Elbow
    set_servo_angle_slow(servo_pins[2], reach_position[3], drop_position[3])  # Wrist
    time.sleep(4)

    print("Moving to Home position...")
    move_stepper(home_position[0])  # Base (stepper motor)
    set_servo_angle_slow(servo_pins[0], 90, home_position[1])  # Shoulder
    set_servo_angle_slow(servo_pins[1], 90, home_position[2])  # Elbow
    set_servo_angle_slow(servo_pins[2], 90, home_position[3])  # Wrist
    time.sleep(5)

    print("Movement complete!")

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    # Cleanup GPIO and servos on exit
    GPIO.cleanup()  # Clean up stepper motor GPIO
    for pin in servo_pins:
        pi.set_servo_pulsewidth(pin, 0)  # Turn off PWM signal for servos
    pi.stop()  # Stop pigpio library
    print("Stepper motor and servos turned off, program exited.")
