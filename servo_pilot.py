import pigpio
import time

class ServoController:
    def __init__(self, servo_pins, initial_angles=None):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise SystemExit("Failed to connect to pigpio daemon")
        
        self.servo_pins = servo_pins
        self.servo_current_angles = initial_angles if initial_angles else [90] * len(servo_pins)
        
        print("Initializing servos...")
        for i, pin in enumerate(self.servo_pins):
            print(f"  Servo {i+1} on GPIO {pin}")
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.set_servo_pulsewidth(pin, 0)
            time.sleep(0.1)  # Small delay between initializations

    def _angle_to_pulse(self, angle):
        pulse = int(500 + (angle / 180.0) * 2000)
        return max(500, min(2500, pulse))

    def move_servos_simultaneously(self, target_angles, step_delay=0.03, step_size=1):
        print("\nMovement command received:")
        print(f"Current angles: {self.servo_current_angles}")
        print(f"Target angles: {target_angles}")
        
        steps_needed = [abs(target - current) for target, current in zip(target_angles, self.servo_current_angles)]
        max_steps = max(int(step / step_size) for step in steps_needed) if any(steps_needed) else 0
        
        print(f"Movement will take {max_steps} steps")
        
        for step in range(max_steps):
            for i in range(len(self.servo_pins)):
                if abs(self.servo_current_angles[i] - target_angles[i]) > 0.1:
                    # Update angle
                    if self.servo_current_angles[i] < target_angles[i]:
                        self.servo_current_angles[i] = min(self.servo_current_angles[i] + step_size, target_angles[i])
                    else:
                        self.servo_current_angles[i] = max(self.servo_current_angles[i] - step_size, target_angles[i])
                    
                    # Set servo position
                    pulse = self._angle_to_pulse(self.servo_current_angles[i])
                    print(f"Step {step}: Servo {i+1} (GPIO {self.servo_pins[i]}) -> Angle: {self.servo_current_angles[i]}°, Pulse: {pulse}µs")
                    self.pi.set_servo_pulsewidth(self.servo_pins[i], pulse)
                    time.sleep(0.01)  # Small delay between servo updates
            
            time.sleep(step_delay)

    def cleanup(self):
        print("\nCleaning up...")
        for pin in self.servo_pins:
            self.pi.set_servo_pulsewidth(pin, 0)
        self.pi.stop()

def main():
    SERVO_PINS = [22, 23, 24]  # GPIO pins for servos
    RESTING_POSITION = [130, 90, 0]
    
    try:
        controller = ServoController(SERVO_PINS, initial_angles=[90, 90, 90])
        
        # Test each servo individually first
        print("\nTesting each servo individually:")
        for i in range(len(SERVO_PINS)):
            test_angles = [90, 90, 90]
            test_angles[i] = 120  # Move just this servo
            print(f"\nMoving only servo {i+1} (GPIO {SERVO_PINS[i]}) to 120°")
            controller.move_servos_simultaneously(test_angles)
            time.sleep(1)
            controller.move_servos_simultaneously([90, 90, 90])
            time.sleep(1)
        
        # Normal operation
        print("\nBeginning normal operation...")
        controller.move_servos_simultaneously(RESTING_POSITION)
        
        while True:
            try:
                target_angles = []
                for i in range(len(SERVO_PINS)):
                    user_input = input(f"Enter angle for Servo {i+1} (0-180, or 'q' to quit): ")
                    if user_input.lower() == 'q':
                        return
                    angle = float(user_input)
                    if not 0 <= angle <= 180:
                        raise ValueError("Angle must be 0-180")
                    target_angles.append(angle)
                
                controller.move_servos_simultaneously(target_angles)
            
            except ValueError as e:
                print(f"Error: {e}. Try again.")
                continue

    except KeyboardInterrupt:
        print("\nProgram interrupted")
    finally:
        controller.cleanup()

if __name__ == "__main__":
    main()