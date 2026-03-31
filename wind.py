import threading
import time
import RPi.GPIO as GPIO
import numpy as np
from servo1 import move_servo_1
from servo2 import move_servo_2
from servo3 import move_servo_3

# === Stepper Motor Setup ===
STEP_PIN = 21
DIR_PIN = 20
LIMIT_SWITCH_PIN = 6
STEPS_PER_REV = 200
STEP_DELAY = 0.03
current_position = 0  # Stepper motor angle in degrees

# === Arm Link Lengths (in mm) ===
A1 = 90
A2 = 140
A4 = 140
A5 = 155  # Not used in IK, might be for end effector?

# === Home/Resting Position ===
HOME_ANGLES = {
    'stepper': 0,
    'servo1': 130,
    'servo2': 90,
    'servo3': 70
}

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def step_motor():
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(STEP_DELAY / 2)
    GPIO.output(STEP_PIN, GPIO.LOW)
    time.sleep(STEP_DELAY / 2)

def calibrate_stepper():
    global current_position
    print("Calibrating stepper...")
    GPIO.output(DIR_PIN, GPIO.LOW)
    while GPIO.input(LIMIT_SWITCH_PIN) == GPIO.HIGH:
        step_motor()
    current_position = 0
    print("Calibration complete. Stepper set to 0°.")

def move_to_angle(target_angle):
    global current_position
    if not 0 <= target_angle <= 180:
        print("Stepper target angle out of bounds (0–180°).")
        return
    angle_diff = target_angle - current_position
    steps = int(STEPS_PER_REV * abs(angle_diff) / 360)
    GPIO.output(DIR_PIN, GPIO.HIGH if angle_diff > 0 else GPIO.LOW)
    for _ in range(steps):
        step_motor()
    current_position = target_angle
    print(f"Stepper moved to {target_angle:.2f}°")

def calculate_inverse_kinematics(x, y, z):
    try:
        theta1 = np.arctan2(y, x)
        planar_dist = np.sqrt(x**2 + y**2)
        d = np.sqrt(planar_dist**2 + z**2)

        phi1 = np.arctan2(z, planar_dist)
        phi2 = np.arccos((A2**2 + d**2 - A4**2) / (2 * A2 * d))
        theta2 = phi1 + phi2

        phi3 = np.arccos((A2**2 + A4**2 - d**2) / (2 * A2 * A4))
        theta3 = np.pi - phi3

        return np.degrees(theta1), np.degrees(theta2), np.degrees(theta3)
    except Exception as e:
        print(f"Inverse kinematics error: {e}")
        return None, None, None

def compensate_tool_offset(x, y, z, theta1_deg, offset_mm=25):
    theta1_rad = np.radians(theta1_deg)
    x_offset = offset_mm * np.cos(theta1_rad)
    y_offset = offset_mm * np.sin(theta1_rad)
    return x - x_offset, y - y_offset, z

def move_all_motors(stepper_angle=None, servo1_angle=None, servo2_angle=None, servo3_angle=None,
                    current_servo1=None, current_servo2=None, current_servo3=None):
    threads = []
    if stepper_angle is not None:
        threads.append(threading.Thread(target=move_to_angle, args=(stepper_angle,)))
    if servo1_angle is not None:
        threads.append(threading.Thread(target=move_servo_1, args=(servo1_angle, current_servo1)))
    if servo2_angle is not None:
        threads.append(threading.Thread(target=move_servo_2, args=(servo2_angle, current_servo2)))
    if servo3_angle is not None:
        threads.append(threading.Thread(target=move_servo_3, args=(servo3_angle, current_servo3)))

    for t in threads:
        t.start()
    for t in threads:
        t.join()

def move_to_home(current_angles):
    print("Returning to home position...")
    move_all_motors(
        stepper_angle=HOME_ANGLES['stepper'],
        servo1_angle=HOME_ANGLES['servo1'],
        servo2_angle=HOME_ANGLES['servo2'],
        servo3_angle=HOME_ANGLES['servo3'],
        current_servo1=current_angles['servo1'],
        current_servo2=current_angles['servo2'],
        current_servo3=current_angles['servo3']
    )
    current_angles.update(HOME_ANGLES)

def main():
    try:
        calibrate_stepper()
        current_angles = {'stepper': 0, 'servo1': 120, 'servo2': 90, 'servo3': 30}

        print("Moving to home position...")
        move_to_home(current_angles)

        while True:
            print("\nEnter coordinates for 4 points or 'q' to quit")
            user_input = input("X1: ")
            if user_input.lower() == 'q':
                break
            try:
                x1 = float(user_input)
                y1 = float(input("Y1: "))
                x2 = float(input("X2: "))
                y2 = float(input("Y2: "))
                x3 = float(input("X3: "))
                y3 = float(input("Y3: "))
                z = float(input("Z (same for all points): "))

                points = [(x1, y1), (x2, y2), (x3, y3)]
                num_steps = 5

                for i in range(len(points) - 1):
                    start = points[i]
                    end = points[i + 1]

                    for step in range(num_steps + 1):
                        t = step / num_steps
                        x = start[0] + (end[0] - start[0]) * t
                        y = start[1] + (end[1] - start[1]) * t

                        theta1_initial, _, _ = calculate_inverse_kinematics(x, y, z)
                        if theta1_initial is None:
                            continue

                        x_corr, y_corr, z_corr = compensate_tool_offset(x, y, z, theta1_initial)
                        theta1, theta2, theta3 = calculate_inverse_kinematics(x_corr, y_corr, z_corr)

                        if None in (theta1, theta2, theta3):
                            print(f"Unreachable point at segment {i+1}, step {step}")
                            continue

                        print(f"Segment {i+1}, Step {step}/{num_steps}: ({x_corr:.1f}, {y_corr:.1f}, {z_corr:.1f})")
                        move_all_motors(
                            stepper_angle=theta1,
                            servo1_angle=theta2,
                            servo2_angle=theta3,
                            servo3_angle=0,
                            current_servo1=current_angles['servo1'],
                            current_servo2=current_angles['servo2'],
                            current_servo3=current_angles['servo3']
                        )
                        current_angles.update({'stepper': theta1, 'servo1': theta2, 'servo2': theta3, 'servo3': 0})
                        time.sleep(0.5)

                move_to_home(current_angles)

            except ValueError:
                print("Invalid input. Please enter numeric values.")
    except KeyboardInterrupt:
        print("\nOperation interrupted by user.")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
