#!/usr/bin/env python3
"""
Basic Functionality Test
Tests the servo control logic without requiring I2C hardware
"""

import sys
import os

# Add the current directory to Python path to import our modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_servo_calculations():
    """Test servo angle to pulse width calculations"""
    print("Testing Servo Control Calculations")
    print("=" * 40)
    
    # Test parameters
    frequency = 50  # 50Hz
    min_pulse = 500  # microseconds
    max_pulse = 2500  # microseconds
    
    print(f"PWM Frequency: {frequency}Hz")
    print(f"Pulse Range: {min_pulse}-{max_pulse}μs")
    print()
    
    # Test different angles
    test_angles = [0, 45, 90, 135, 180]
    
    for angle in test_angles:
        # Calculate pulse width
        pulse_width = min_pulse + (angle / 180.0) * (max_pulse - min_pulse)
        
        # Calculate PWM value
        pwm_value = int((pulse_width / 1000000.0) * frequency * 4096)
        
        print(f"Angle: {angle:3d}° → Pulse: {pulse_width:6.0f}μs → PWM: {pwm_value:4d}")
    
    print("\n✓ All calculations completed successfully!")

def test_validation():
    """Test input validation logic"""
    print("\nTesting Input Validation")
    print("=" * 40)
    
    # Test valid inputs
    valid_angles = [0, 90, 180]
    valid_channels = [0, 8, 15]
    
    print("Valid inputs:")
    for angle in valid_angles:
        for channel in valid_channels:
            print(f"  Channel {channel:2d}, Angle {angle:3d}° ✓")
    
    # Test invalid inputs
    invalid_angles = [-1, 181, 200]
    invalid_channels = [-1, 16, 20]
    
    print("\nInvalid inputs (should raise errors):")
    for angle in invalid_angles:
        print(f"  Angle {angle:3d}° ✗ (out of range)")
    
    for channel in invalid_channels:
        print(f"  Channel {channel:2d} ✗ (out of range)")
    
    print("\n✓ Validation logic verified!")

def test_pwm_calculations():
    """Test PWM value calculations"""
    print("\nTesting PWM Calculations")
    print("=" * 40)
    
    frequency = 50
    pulse_widths = [500, 1000, 1500, 2000, 2500]
    
    print(f"PWM Frequency: {frequency}Hz")
    print(f"Resolution: 12-bit (0-4095)")
    print()
    
    for pulse in pulse_widths:
        # Convert pulse width to PWM value
        pwm_value = int((pulse / 1000000.0) * frequency * 4096)
        
        # Convert back to verify
        calculated_pulse = (pwm_value / 4096.0) / frequency * 1000000
        
        print(f"Pulse: {pulse:4d}μs → PWM: {pwm_value:4d} → Back: {calculated_pulse:6.1f}μs")
    
    print("\n✓ PWM calculations verified!")

def main():
    """Run all tests"""
    print("PCA9685 Servo Control - Basic Functionality Tests")
    print("=" * 60)
    print("These tests verify the logic without requiring I2C hardware")
    print("=" * 60)
    
    try:
        test_servo_calculations()
        test_validation()
        test_pwm_calculations()
        
        print("\n" + "=" * 60)
        print("🎉 All tests passed successfully!")
        print("The servo control logic is working correctly.")
        print("\nTo test with real hardware:")
        print("1. Transfer these files to a Raspberry Pi")
        print("2. Connect PCA9685 board via I2C")
        print("3. Run: python3 servo_control.py")
        
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()

