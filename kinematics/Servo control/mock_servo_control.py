#!/usr/bin/env python3
"""
Mock PCA9685 Servo Control Script
Simulates servo control for testing on systems without I2C hardware
"""

import time
import math
import sys

class MockPCA9685:
    """Mock PCA9685 PWM controller class for testing without hardware"""
    
    def __init__(self, bus_number=1, address=0x40):
        """
        Initialize mock PCA9685 controller
        
        Args:
            bus_number (int): I2C bus number (simulated)
            address (int): I2C address of PCA9685 (simulated)
        """
        self.bus_number = bus_number
        self.address = address
        self.frequency = 50  # 50Hz for servos
        self.servo_positions = [90] * 16  # Track servo positions
        
        print(f"🔧 Mock PCA9685 initialized on bus {bus_number}, address 0x{address:02X}")
        print("⚠️  This is a simulation - no actual hardware is being controlled")
    
    def set_pwm_frequency(self, frequency):
        """Set PWM frequency in Hz (simulated)"""
        self.frequency = frequency
        print(f"🔧 PWM frequency set to {frequency}Hz")
    
    def set_pwm(self, channel, on, off):
        """Set PWM output on a specific channel (simulated)"""
        if channel < 0 or channel > 15:
            raise ValueError("Channel must be between 0 and 15")
        
        if on < 0 or on > 4095 or off < 0 or off > 4095:
            raise ValueError("PWM values must be between 0 and 4095")
        
        print(f"🔧 Channel {channel}: PWM set to ON={on}, OFF={off}")
    
    def set_servo_angle(self, channel, angle, min_pulse=500, max_pulse=2500):
        """Set servo angle on a specific channel (simulated)"""
        if angle < 0 or angle > 180:
            raise ValueError("Angle must be between 0 and 180 degrees")
        
        # Convert angle to pulse width
        pulse_width = min_pulse + (angle / 180.0) * (max_pulse - min_pulse)
        
        # Convert pulse width to PWM value
        pwm_value = int((pulse_width / 1000000.0) * self.frequency * 4096)
        
        # Update servo position
        self.servo_positions[channel] = angle
        
        print(f"🔧 Channel {channel}: Servo moved to {angle}° (pulse: {pulse_width:.0f}μs, PWM: {pwm_value})")
        
        # Simulate servo movement time
        time.sleep(0.1)
    
    def set_servo_pulse(self, channel, pulse_width):
        """Set servo pulse width directly (simulated)"""
        # Convert pulse width to angle (approximate)
        angle = int((pulse_width - 500) / 2000 * 180)
        angle = max(0, min(180, angle))
        
        # Update servo position
        self.servo_positions[channel] = angle
        
        print(f"🔧 Channel {channel}: Servo pulse set to {pulse_width}μs (approx {angle}°)")
        time.sleep(0.1)
    
    def set_all_servos(self, angles, min_pulse=500, max_pulse=2500):
        """Set multiple servos at once (simulated)"""
        for channel, angle in enumerate(angles):
            if channel < 16:
                self.set_servo_angle(channel, angle, min_pulse, max_pulse)
    
    def stop_servo(self, channel):
        """Stop servo on a specific channel (simulated)"""
        print(f"🔧 Channel {channel}: Servo stopped")
    
    def stop_all_servos(self):
        """Stop all servos (simulated)"""
        print("🔧 All servos stopped")
    
    def get_servo_position(self, channel):
        """Get current servo position (simulated)"""
        if 0 <= channel < 16:
            return self.servo_positions[channel]
        return None
    
    def close(self):
        """Close mock controller (simulated)"""
        print("🔧 Mock PCA9685 controller closed")

def main():
    """Main function demonstrating mock servo control"""
    print("Mock PCA9685 Servo Control Demo")
    print("=" * 40)
    print("⚠️  This is a simulation for testing purposes")
    print("   To control real hardware, use this on a Raspberry Pi")
    print("=" * 40)
    
    try:
        # Initialize mock PCA9685 controller
        print("\nInitializing mock PCA9685 controller...")
        controller = MockPCA9685(bus_number=1, address=0x40)
        print("Controller initialized successfully!")
        
        # Test servo on channel 0
        channel = 0
        print(f"\nTesting servo on channel {channel}")
        
        # Move servo to center position
        print("Moving servo to center (90 degrees)...")
        controller.set_servo_angle(channel, 90)
        time.sleep(0.5)
        
        # Move servo to minimum position
        print("Moving servo to minimum (0 degrees)...")
        controller.set_servo_angle(channel, 0)
        time.sleep(0.5)
        
        # Move servo to maximum position
        print("Moving servo to maximum (180 degrees)...")
        controller.set_servo_angle(channel, 180)
        time.sleep(0.5)
        
        # Return to center
        print("Returning servo to center...")
        controller.set_servo_angle(channel, 90)
        time.sleep(0.5)
        
        # Sweep servo back and forth
        print("\nPerforming servo sweep...")
        for angle in range(0, 181, 20):
            controller.set_servo_angle(channel, angle)
            time.sleep(0.1)
        
        for angle in range(180, -1, -20):
            controller.set_servo_angle(channel, angle)
            time.sleep(0.1)
        
        # Return to center
        controller.set_servo_angle(channel, 90)
        
        # Test multiple servos
        print("\nTesting multiple servos...")
        angles = [90, 45, 135, 0, 180, 90]
        controller.set_all_servos(angles)
        
        # Show current positions
        print("\nCurrent servo positions:")
        for i in range(6):
            pos = controller.get_servo_position(i)
            print(f"  Channel {i}: {pos}°")
        
        print("\nDemo completed successfully!")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"Error during demo: {e}")
    finally:
        # Clean up
        if 'controller' in locals():
            controller.stop_all_servos()
            controller.close()
        print("Controller cleaned up")

if __name__ == "__main__":
    main()

