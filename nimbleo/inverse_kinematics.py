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
servo_pins = [22, 23, 24]  # GPIO pins for the 3 servos
servo_current_angles = [0, 0, 0]  # Store the current angle of each servo

# === Robotic Arm Link Lengths ===
L1 = 52  # Base to shoulder
L2 = 140  # Shoulder to elbow
L3 = 140  # Elbow to wrist

# Function to convert degrees to radians
def deg_to_rad(deg):
    return deg * (math.pi / 180.0)

# Function to compute forward kinematics
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

# Function to compute inverse kinematics
def inverse_kinematics(x, y, z):
    theta1 = math.atan2(y, x) * (180.0 / math.pi)  # Base angle (theta1)

    wrist_z = z - L1
    wrist_xy = math.sqrt(x**2 + y**2)
    wrist_dist = math.sqrt(wrist_xy**2 + wrist_z**2)

    if wrist_dist > (L2 + L3):
        print("Position out of reach.")
        return None

    cos_angle2 = (L2**2 + wrist_dist**2 - L3**2) / (2 * L2 * wrist_dist)
    sin_angle2 = math.sqrt(1 - cos_angle2**2)
    theta2 = math.atan2(wrist_z, wrist_xy) + math.atan2(sin_angle2, cos_angle2)

    cos_angle3 = (L2**2 + L3**2 - wrist_dist**2) / (2 * L2 * L3)
    sin_angle3 = math.sqrt(1 - cos_angle3**2)
    theta3 = math.atan2(sin_angle3, cos_angle3)

    theta2 = theta2 * (180.0 / math.pi)
    theta3 = theta3 * (180.0 / math.pi)
    theta4 = 0  # No wrist rotation

    return theta1, theta2, theta3, theta4

# Function to move servos smoothly
def set_servo_angle_slow(pin, start_angle, end_angle, step_delay=0.05):
    step = 1 if start_angle < end_angle else -1
    for angle in range(int(start_angle), int(end_angle), step):
        pulse_width = int(500 + (angle / 180.0) * 2000)
        pi.set_servo_pulsewidth(pin, pulse_width)
        time.sleep(step_delay)
    pulse_width = int(500 + (end_angle / 180.0) * 2000)
    pi.set_servo_pulsewidth(pin, pulse_width)

# Stepper motor calibration
def calibrate_stepper():
    global current_position, servo_current_angles
    print("Calibrating stepper motor...")

    GPIO.output(DIR_PIN, GPIO.LOW)
    while GPIO.input(LIMIT_SWITCH_PIN) == GPIO.HIGH:
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.01)
    current_position = 0
    print("Calibration complete. 0-degree position set.")

    for i in range(len(servo_pins)):
        set_servo_angle_slow(servo_pins[i], servo_current_angles[i], 0)
        servo_current_angles[i] = 0

# Move stepper motor to target angle
def move_stepper(target_angle):
    global current_position
    min_allowed_angle = current_position - MAX_ANGLE
    max_allowed_angle = current_position + MAX_ANGLE

    if target_angle < min_allowed_angle:
        target_angle = min_allowed_angle
    elif target_angle > max_allowed_angle:
        target_angle = max_allowed_angle

    angle_to_move = target_angle - current_position
    steps = int((STEPS_PER_REV * abs(angle_to_move)) / 360)

    if angle_to_move > 0:
        GPIO.output(DIR_PIN, GPIO.HIGH)
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)

    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.01)

    current_position = target_angle
    print(f"Stepper moved to: {current_position} degrees")

# === Main Program ===
try:
    home_position = [0, 150, 180, 0]
    calibrate_stepper()

    while True:
        print("Enter end-effector position (x, y, z):")
        x = float(input("x: "))
        y = float(input("y: "))
        z = float(input("z: "))

        joint_angles = inverse_kinematics(x, y, z)
        if joint_angles is None:
            print("Position unreachable.")
            continue

        base_angle, shoulder_angle, elbow_angle, wrist_angle = joint_angles
        print(f"Moving to position x={x}, y={y}, z={z} with angles: Base={base_angle}, Shoulder={shoulder_angle}, Elbow={elbow_angle}, Wrist={wrist_angle}")

        move_stepper(base_angle)
        set_servo_angle_slow(servo_pins[0], servo_current_angles[0], shoulder_angle)
        set_servo_angle_slow(servo_pins[1], servo_current_angles[1], elbow_angle)
        set_servo_angle_slow(servo_pins[2], servo_current_angles[2], wrist_angle)

        servo_current_angles = [shoulder_angle, elbow_angle, wrist_angle]

except KeyboardInterrupt:
    print("Program interrupted.")

finally:
    GPIO.cleanup()
    for pin in servo_pins:
        pi.set_servo_pulsewidth(pin, 0)
    pi.stop()
    print("Stepper and servos turned off. Exiting.")
