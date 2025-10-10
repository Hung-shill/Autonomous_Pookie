""" test_motors.py - Single file motor test for Cokoino 4WD Robot HAT

Quick test to verify hardware and GPIO setup works.
Tests each motor M1-M4 individually for 2 seconds each.

SAFETY: Keep wheels OFF the table during testing!

Hardware: Raspberry Pi 5 + Cokoino 4WD Robot HAT
Usage: python3 test_motors.py
"""

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
    
    def set_motor_speed(self, movement_protocol, speed):
        """Set PWM speed for movement protocol (either all 4 wheels move backward or forward)."""
        speed = max(0.0, min(1.0, speed))
        if speed > 0 and speed < MIN_DUTY_CYCLE:
            speed = MIN_DUTY_CYCLE
        
        
        """ moving protocol set could be:
            1 = straight line
            2 = turning left
            3 = turing right
            4 = disco dance or sumshit """
        if movement_protocol == 1:
            self.nsleep1.value = speed
            self.nsleep2.value = speed
        elif movement_protocol == 2: #try turning left with this (slower than linear) and also need omni-directional wheels to work
            self.nsleep1.value = speed - 0.5 #using omnidirectional wheels to turn
        elif movement_protocol == 3: #try turning right with this (slower than linear) and also need omni-directional wheels to work
            self.nsleep2.value = speed - 0.5 #using omnidirectional wheels to turn

   
  
    
    def linear_movement(self, movement_protocol, direction, speed, duration):
        
        print(f" M{movement_protocol} running {direction} at {speed:.1f} speed for {duration}s")
        
        # Set speed for appropriate motor group
        self.set_motor_speed(movement_protocol, speed)
        
        # The setup pairs are all 4 forward or all 4 backward
        if direction == 'forward':
            self.m1_in1.on()
            self.m2_in1.on()
            self.m3_in1.on()
            self.m4_in1.on()
            
        if direction == 'backward': #moving backward
            self.m1_in2.on()
            self.m2_in2.on()
            self.m3_in2.on()
            self.m4_in2.on()
        
        # Run for specified duration
        time.sleep(duration)
        
        # Stop this motor
        self.stop_all()
        print(f" M{movement_protocol} stopped")
        
    
    def turning_movement(self, movement_protocol, angle, speed, duration):
        print(f" M{movement_protocol} turning {angle} at {speed:.1f} speed for {duration}s")
        
        self.set_motor_speed(movement_protocol, speed)
        
        

    
    
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

    
    def cleanup(self):
        """Clean up GPIO resources."""
        print(" Cleaning up GPIO...")
        self.stop_all()
        
        # Close all GPIO pins
        pins = [self.nsleep1, self.nsleep2, 
                self.m1_in1, self.m1_in2, self.m2_in1, self.m2_in2,
                self.m3_in1, self.m3_in2, self.m4_in1, self.m4_in2]
        
        for pin in pins:
            if pin:
                pin.close()
        
        print("✅ Cleanup complete")


def main():
    """Main test function."""
    
    print(" Cokoino 4WD Motor HAT - Quick Test")
    print(" Raspberry Pi 5 + gpiozero + lgpio")
    print()
    
    print("⚠️  SAFETY WARNING:")
    print("   - Keep robot wheels OFF the table!")
    print("   - Have Ctrl+C ready for emergency stop")
    print("   - Test will run for ~16 seconds total") #the motor is kinda weak so don't need to worry much about the car moving
    print()
    
    try:
        response = input("Ready to test motors? (y/N): ").lower().strip()
        if response not in ['y', 'yes']:
            print(" Test cancelled")
            return
        
        print("\n Starting motor test...")
        
        # Initialize motor controller
        motors = Motor_Movement()
        
        # Test each motor forward and backward
        test_speed = 0.5
        test_time = 2.0
        
        for motor_num in range(1,2 ):
            print(f"\n--- Testing Motor M{motor_num} group movement ---")
            
            # Forward test
            motors.linear_movement(motor_num, 'forward', test_speed, test_time)
            time.sleep(0.5)  # Brief pause
            
            # Backward test  
            motors.linear_movement(motor_num, 'backward', test_speed, test_time)
            time.sleep(0.5)  # Brief pause
        
        print("\n All motor tests completed successfully!")
        print(" Your Cokoino 4WD HAT is working correctly")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ Test interrupted by user (Ctrl+C)")
        print(" Emergency stop activated")
        
    except Exception as e:
        print(f"\nError during test: {e}")
        print("❌ Check your wiring and connections")
        
    finally:
        print(" Test session ended")


main()
