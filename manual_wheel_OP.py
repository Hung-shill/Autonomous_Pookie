"""
Differential Speed Robot Control with Keyboard
Cokoino 4WD Robot HAT + Raspberry Pi 5

Controls:
  W = Forward (straight)
  S = Backward (straight)
  A = Arc Left (turns while moving forward)
  D = Arc Right (turns while moving forward)
  Space = Stop
  ESC = Quit

Hardware: Raspberry Pi 5 + Cokoino 4WD Robot HAT
Usage: python3 wheel_operation.py
"""

import time
import atexit
import sys
import tty
import termios
import atexit
from gpiozero import Device, PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

# Set lgpio backend for Pi 5 compatibility
Device.pin_factory = LGPIOFactory()

# GPIO pin assignments from Cokoino 4WD Robot HAT
NSLEEP1_PIN = 12    # Master enable for M1+M2
NSLEEP2_PIN = 13    # Master enable for M3+M4

M1_AIN1_PIN = 17    # Motor 1 direction pins
M1_AIN2_PIN = 27
M2_BIN1_PIN = 22    # Motor 2 direction pins
M2_BIN2_PIN = 23
M3_AIN1_PIN = 24    # Motor 3 direction pins
M3_AIN2_PIN = 25
M4_BIN1_PIN = 26    # Motor 4 direction pins
M4_BIN2_PIN = 16

MIN_DUTY_CYCLE = 0.35  # Minimum PWM to overcome motor stall
TURN_RATIO = 0.4       # Speed ratio for slower side when turning

class Motor_Movement:
    """Motor movement for Cokoino 4WD HAT."""
    
    def __init__(self):
        """Initialize all GPIO pins."""
        print("Initializing GPIO pins...")
        
        # PWM controllers for speed (NSLEEP pins)
        self.nsleep1 = PWMOutputDevice(NSLEEP1_PIN, frequency=500)
        self.nsleep2 = PWMOutputDevice(NSLEEP2_PIN, frequency=500)
        
        # Direction control pins for each motor
        self.m1_in1 = DigitalOutputDevice(M1_AIN1_PIN, initial_value=False)
        self.m1_in2 = DigitalOutputDevice(M1_AIN2_PIN, initial_value=False)
        
        self.m2_in1 = DigitalOutputDevice(M2_BIN1_PIN, initial_value=False)
        self.m2_in2 = DigitalOutputDevice(M2_BIN2_PIN, initial_value=False)
        
        self.m3_in1 = DigitalOutputDevice(M3_AIN1_PIN, initial_value=False)
        self.m3_in2 = DigitalOutputDevice(M3_AIN2_PIN, initial_value=False)
        
        self.m4_in1 = DigitalOutputDevice(M4_BIN1_PIN, initial_value=False)
        self.m4_in2 = DigitalOutputDevice(M4_BIN2_PIN, initial_value=False)
        
        # Stop everything initially
        self.stop_all()
        
        # Register cleanup on exit
        atexit.register(self.cleanup)
        
        print("GPIO initialized successfully")
    
    def _normalize_speed(self, speed):
        """Normalize speed to valid range with minimum duty cycle."""
        speed = max(0.0, min(1.0, abs(speed)))
        if speed > 0 and speed < MIN_DUTY_CYCLE:
            speed = MIN_DUTY_CYCLE
        return speed

    def _set_motor_speeds(self, left_speed, right_speed):
        """Set PWM speeds for left and right motor groups."""
        self.nsleep1.value = self._normalize_speed(left_speed)
        self.nsleep2.value = self._normalize_speed(right_speed)
    
    def _set_direction_forward(self):
        """Set all motors to forward direction."""
        self.m1_in1.on()
        self.m1_in2.off()
        self.m2_in1.on()
        self.m2_in2.off()
        self.m3_in1.on()
        self.m3_in2.off()
        self.m4_in1.on()
        self.m4_in2.off()
    
    def _set_direction_backward(self):
        """Set all motors to backward direction."""
        self.m1_in1.off()
        self.m1_in2.on()
        self.m2_in1.off()
        self.m2_in2.on()
        self.m3_in1.off()
        self.m3_in2.on()
        self.m4_in1.off()
        self.m4_in2.on()
    
    def stop_all(self):
        """Stop all motors immediately."""
        print(" Stopping all motors")
        
        # Stop all direction pins
        self.m1_in1.off()
        self.m1_in2.off()
        self.m2_in1.off()
        self.m2_in2.off()
        self.m3_in1.off()
        self.m3_in2.off()
        self.m4_in1.off()
        self.m4_in2.off()

    def forward(self, speed=1.0):
        """Move straight forward"""
        self._set_motor_speeds(speed, speed)
        self._set_direction_forward()
        print("Forward")
    
    def backward(self, speed=1.0):
        """Move straight backward"""
        self._set_motor_speeds(speed, speed)
        self._set_direction_backward()
        print("Backward")

    def turning_left(self, speed=0.6):
        """Turn left in an arc - left side slower, right side normal speed."""
        left_speed = speed * TURN_RATIO   # Left side goes slower
        right_speed = speed               # Right side at full speed
        
        self._set_motor_speeds(left_speed, right_speed)
        self._set_direction_forward()
        print(f"↶  Arc Left (L:{left_speed:.2f} R:{right_speed:.2f})")
    
    def turning_right(self, speed=0.6):
        """Turn right in an arc - right side slower, left side normal speed."""
        left_speed = speed                # Left side at full speed
        right_speed = speed * TURN_RATIO  # Right side goes slower
        
        self._set_motor_speeds(left_speed, right_speed)
        self._set_direction_forward()
        print(f"↷  Arc Right (L:{left_speed:.2f} R:{right_speed:.2f})")

    def cleanup(self):
            """Clean up GPIO resources."""
            print("\n🧹 Cleaning up GPIO...")
            self.stop_all()
            
            # Close all GPIO pins
            pins = [self.nsleep1, self.nsleep2, 
                    self.m1_in1, self.m1_in2, self.m2_in1, self.m2_in2,
                    self.m3_in1, self.m3_in2, self.m4_in1, self.m4_in2]
            
            for pin in pins:
                if pin:
                    pin.close()
            
            print("✅ Cleanup complete")

def get_key():
    """Get single keypress from terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def main():
    """Main keyboard control loop."""
    print("\n" + "="*50)
    print("🚗 DIFFERENTIAL SPEED ROBOT CONTROL")
    print("   Raspberry Pi 5 + Cokoino 4WD HAT")
    print("="*50)
    print("\n📋 Controls:")
    print("  W       = Forward (straight)")
    print("  S       = Backward (straight)")
    print("  A       = Arc Left (curved turn)")
    print("  D       = Arc Right (curved turn)")
    print("  Space   = Stop")
    print("  ESC     = Quit")
    print("\n💡 Turns in smooth arcs like a car!")
    print(f"   Turn ratio: {TURN_RATIO} (slower side)")
    print("\n  Keep wheels OFF the table!")
    print("="*50 + "\n")
    
    try:
            response = input("Ready to start? (y/N): ").lower().strip()
            if response not in ['y', 'yes']:
                print("❌ Cancelled")
                return
            
            print("\n🚀 Starting keyboard control...")
            print("Press keys to move (no need to press Enter)\n")
            
            motors = Motor_Movement()
            
            while True:
                key = get_key().lower()
                
                if key == 'w':
                    motors.forward()
                elif key == 's':
                    motors.backward()
                elif key == 'a':
                    motors.turning_left()
                elif key == 'd':
                    motors.turning_right()
                elif key == ' ':
                    motors.stop_all()
                    print("🛑 Stop")
                elif key == '\x1b':  # ESC key
                    print("\n👋 Exiting...")
                    break
            
    except KeyboardInterrupt:
        print("\n\n Interrupted by Ctrl+C")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        print("\n🏁 Program ended")


if __name__ == "__main__":
    main()