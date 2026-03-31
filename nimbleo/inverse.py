import cv2
import numpy as np
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

# === OpenCV Setup ===
cap = cv2.VideoCapture(0)
min_area_threshold = 500  # Minimum area to detect an object
scaling_factor = 0.512
x_offset = -70.0
y_offset = 120.0

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

# Move Stepper Motor to a Target Angle
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
        GPIO.output(DIR_PIN, GPIO.HIGH)  # Move forward
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)   # Move backward

    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.01)

    current_position = target_angle
    print(f"Stepper moved to: {current_position} degrees")

# Inverse Kinematics Function to calculate θ1, θ2, θ3
def inverse_kinematics(x, y, z):
    theta_1 = math.atan2(y, x) * (180 / math.pi)  # Base angle

    d_wrist = math.sqrt(x**2 + y**2) - L4  # Distance to wrist position
    z_wrist = z - L1  # Adjust Z for base height

    D = (d_wrist**2 + z_wrist**2 - L2**2 - L3**2) / (2 * L2 * L3)
    theta_3 = math.acos(D) * (180 / math.pi)

    theta_2 = math.atan2(z_wrist, d_wrist) - math.atan2(L3 * math.sin(math.radians(theta_3)), L2 + L3 * math.cos(math.radians(theta_3)))
    theta_2 = theta_2 * (180 / math.pi)

    theta_2 = abs(theta_2)
    return theta_1, theta_2, theta_3

# Process frames and move arm based on object detection
def process_frame_and_move_arm(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area_threshold:
            x, y, w, h = cv2.boundingRect(contour)
            object_center_x = x + w / 2
            object_center_y = y + h / 2

            # Convert detected position to arm coordinates (x, y, z)
            arm_x = (object_center_x * scaling_factor) + x_offset
            arm_y = (object_center_y * scaling_factor) + y_offset
            arm_z = 100  # Example fixed Z

            # Use inverse kinematics to get joint angles
            theta_1, theta_2, theta_3 = inverse_kinematics(arm_x, arm_y, arm_z)

            # Move the stepper (base) and servos (shoulder, elbow)
            move_stepper(theta_1)
            set_servo_angle_slow(servo_pins[0], servo_current_angles[0], theta_2)
            set_servo_angle_slow(servo_pins[1], servo_current_angles[1], theta_3)

            servo_current_angles[0] = theta_2
            servo_current_angles[1] = theta_3

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    process_frame_and_move_arm(frame)
    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

pi.stop()
GPIO.cleanup()
