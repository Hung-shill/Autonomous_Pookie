import time
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


class SimpleMotorTest:
    """Simple motor tester for Cokoino 4WD HAT."""
    
    def __init__(self):
        """Initialize all GPIO pins."""
        print("๐ง Initializing GPIO pins...")
        
        # PWM controllers for speed (NSLEEP pins)
        self.nsleep1 = PWMOutputDevice(NSLEEP1_PIN, frequency=1000)
        self.nsleep2 = PWMOutputDevice(NSLEEP2_PIN, frequency=1000)
        
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
        
        print("โ GPIO initialized successfully")
    
    def set_motor_speed(self, motor_group, speed):
        """Set PWM speed for motor group (1=M1+M2, 2=M3+M4)."""
        speed = max(0.0, min(1.0, speed))
        if speed > 0 and speed < MIN_DUTY_CYCLE:
            speed = MIN_DUTY_CYCLE
            
        if motor_group == 1:
            self.nsleep1.value = speed
        elif motor_group == 2:
            self.nsleep2.value = speed
    
    def run_motor(self, motor_num, direction, speed=0.6, duration=2.0):
        """
        Run specific motor in given direction.
        
        Args:
            motor_num (int): Motor number (1-4)
            direction (str): 'forward' or 'backward'
            speed (float): Speed 0.0-1.0
            duration (float): Run time in seconds
        """
        print(f"๐ M{motor_num} running {direction} at {speed:.1f} speed for {duration}s")
        
        # Set speed for appropriate motor group
        motor_group = 1 if motor_num <= 2 else 2
        self.set_motor_speed(motor_group, speed)
        
        # Set direction pins
        if motor_num == 1:
            if direction == 'forward':
                self.m1_in1.on()
                self.m1_in2.off()
            else:  # backward
                self.m1_in1.off()
                self.m1_in2.on()
                
        elif motor_num == 2:
            if direction == 'forward':
                self.m2_in1.on()
                self.m2_in2.off()
            else:  # backward
                self.m2_in1.off()
                self.m2_in2.on()
                
        elif motor_num == 3:
            if direction == 'forward':
                self.m3_in1.on()
                self.m3_in2.off()
            else:  # backward
                self.m3_in1.off()
                self.m3_in2.on()
                
        elif motor_num == 4:
            if direction == 'forward':
                self.m4_in1.on()
                self.m4_in2.off()
            else:  # backward
                self.m4_in1.off()
                self.m4_in2.on()
        
        # Run for specified duration
        time.sleep(duration)
        
        # Stop this motor
        self.stop_motor(motor_num)
        print(f"โน M{motor_num} stopped")
    
    def stop_motor(self, motor_num):
        """Stop specific motor."""
        if motor_num == 1:
            self.m1_in1.off()
            self.m1_in2.off()
        elif motor_num == 2:
            self.m2_in1.off()
            self.m2_in2.off()
        elif motor_num == 3:
            self.m3_in1.off()
            self.m3_in2.off()
        elif motor_num == 4:
            self.m4_in1.off()
            self.m4_in2.off()
            
        # If both motors in group are stopped, disable PWM
        if motor_num <= 2:
            # Check if both M1 and M2 are stopped
            if (not self.m1_in1.value and not self.m1_in2.value and 
                not self.m2_in1.value and not self.m2_in2.value):
                self.set_motor_speed(1, 0)
        else:
            # Check if both M3 and M4 are stopped  
            if (not self.m3_in1.value and not self.m3_in2.value and
                not self.m4_in1.value and not self.m4_in2.value):
                self.set_motor_speed(2, 0)
    
    def stop_all(self):
        """Stop all motors immediately."""
        print("๐ Stopping all motors")
        
        # Stop all direction pins
        self.m1_in1.off()
        self.m1_in2.off()
        self.m2_in1.off()
        self.m2_in2.off()
        self.m3_in1.off()
        self.m3_in2.off()
        self.m4_in1.off()
        self.m4_in2.off()
        
        # Disable both PWM groups
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)
    
    def cleanup(self):
        """Clean up GPIO resources."""
        print("๐งน Cleaning up GPIO...")
        self.stop_all()
        
        # Close all GPIO pins
        pins = [self.nsleep1, self.nsleep2, 
                self.m1_in1, self.m1_in2, self.m2_in1, self.m2_in2,
                self.m3_in1, self.m3_in2, self.m4_in1, self.m4_in2]
        
        for pin in pins:
            if pin:
                pin.close()
        
        print("โ Cleanup complete")


def main():
    """Main test function."""
    
    print("๐ค Cokoino 4WD Motor HAT - Quick Test")
    print("๐ง Raspberry Pi 5 + gpiozero + lgpio")
    print()
    
    print("โ ๏ธ  SAFETY WARNING:")
    print("   - Keep robot wheels OFF the table!")
    print("   - Have Ctrl+C ready for emergency stop")
    print("   - Test will run for ~16 seconds total")
    print()
    
    try:
        response = input("Ready to test motors? (y/N): ").lower().strip()
        if response not in ['y', 'yes']:
            print("๐ Test cancelled")
            return
        
        print("\n๐ Starting motor test...")
        
        # Initialize motor controller
        motors = SimpleMotorTest()
        
        # Test each motor forward and backward
        test_speed = 0.6
        test_time = 2.0
        
        for motor_num in range(1, 5):
            print(f"\n--- Testing Motor M{motor_num} ---")
            
            # Forward test
            motors.run_motor(motor_num, 'forward', test_speed, test_time)
            time.sleep(0.5)  # Brief pause
            
            # Backward test  
            motors.run_motor(motor_num, 'backward', test_speed, test_time)
            time.sleep(0.5)  # Brief pause
        
        print("\n๐ All motor tests completed successfully!")
        print("โ Your Cokoino 4WD HAT is working correctly")
        
    except KeyboardInterrupt:
        print("\n\nโ ๏ธ Test interrupted by user (Ctrl+C)")
        print("๐ Emergency stop activated")
        
    except Exception as e:
        print(f"\n๐ฅ Error during test: {e}")
        print("โ Check your wiring and connections")
        
    finally:
        print("๐ Test session ended")


if __name__ == "__main__":
    main()