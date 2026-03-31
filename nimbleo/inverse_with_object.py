import RPi.GPIO as GPIO
import pigpio
import cv2
import numpy as np
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
        print(f"Target angle is below the allowable limit. Moving to {min_allowed_angle} degrees instead.")
        target_angle = min_allowed_angle
    elif target_angle > max_allowed_angle:
        print(f"Target angle is above the allowable limit. Moving to {max_allowed_angle} degrees instead.")
        target_angle = max_allowed_angle

    angle_to_move = target_angle - current_position
    if angle_to_move == 0:
        print("No movement required.")
        return

    steps = int((STEPS_PER_REV * abs(angle_to_move)) / 360)

    if angle_to_move > 0:
        GPIO.output(DIR_PIN, GPIO.HIGH)
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)

    for i in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(0.01)

    current_position = target_angle
    print(f"Stepper moved to: {current_position} degrees")

# Inverse Kinematics Function
def inverse_kinematics(x, y, z):
    theta_1 = math.atan2(y, x) * (180 / math.pi)
    d_wrist = math.sqrt(x**2 + y**2) - L4
    z_wrist = z - L1

    D = (d_wrist**2 + z_wrist**2 - L2**2 - L3**2) / (2 * L2 * L3)
    theta_3 = math.acos(D) * (180 / math.pi)

    theta_2 = math.atan2(z_wrist, d_wrist) - math.atan2(L3 * math.sin(math.radians(theta_3)), L2 + L3 * math.cos(math.radians(theta_3)))
    theta_2 = theta_2 * (180 / math.pi)
    theta_2 = abs(theta_2)

    print(f"Inverse Kinematics - θ1: {theta_1:.2f}, θ2: {theta_2:.2f}, θ3: {theta_3:.2f}")
    return theta_1, theta_2, theta_3

# Object Detection and Position Extraction
def detect_object():
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([30, 150, 50])
        upper_bound = np.array([70, 255, 255])

        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 1000:
                x, y, w, h = cv2.boundingRect(contour)
                cx, cy = x + w // 2, y + h // 2
                print(f"Object detected at ({cx}, {cy})")
                return cx, cy

        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Main Program
try:
    calibrate_stepper()
    z_fixed = 0
    x, y = detect_object()
    theta_1, theta_2, theta_3 = inverse_kinematics(x, y, z_fixed)
    move_stepper(theta_1)
    set_servo_angle_slow(servo_pins[1], servo_current_angles[1], theta_3)
    set_servo_angle_slow(servo_pins[0], servo_current_angles[0], theta_2)
except KeyboardInterrupt:
    print("Program interrupted.")
finally:
    pi.stop()
    GPIO.cleanup()
