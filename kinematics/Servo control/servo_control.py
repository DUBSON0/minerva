#!/usr/bin/env python3
"""
PCA9685 Servo Control Script
Controls servo motors connected to PCA9685 16-channel 12-bit PWM driver
Compatible with Raspberry Pi and Ubuntu systems
"""

import time
import math
from smbus2 import SMBus
import sys

class PCA9685:
    """PCA9685 PWM controller class for servo control"""
    
    # PCA9685 registers
    MODE1 = 0x00
    MODE2 = 0x01
    SUBADR1 = 0x02
    SUBADR2 = 0x03
    SUBADR3 = 0x04
    ALLCALLADR = 0x05
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09
    ALL_LED_ON_L = 0xFA
    ALL_LED_ON_H = 0xFB
    ALL_LED_OFF_L = 0xFC
    ALL_LED_OFF_H = 0xFD
    PRESCALE = 0xFE
    
    def __init__(self, bus_number=1, address=0x40):
        """
        Initialize PCA9685 controller
        
        Args:
            bus_number (int): I2C bus number (usually 1 for Raspberry Pi)
            address (int): I2C address of PCA9685 (default 0x40)
        """
        self.bus_number = bus_number
        self.address = address
        self.bus = None
        self.frequency = 50  # 50Hz for servos
        
        try:
            self.bus = SMBus(bus_number)
            self._initialize()
        except Exception as e:
            print(f"Error initializing I2C bus: {e}")
            sys.exit(1)
    
    def _initialize(self):
        """Initialize the PCA9685 controller"""
        # Reset the controller
        self._write_byte(self.MODE1, 0x00)
        time.sleep(0.1)
        
        # Set PWM frequency
        self.set_pwm_frequency(self.frequency)
        
        # Enable auto-increment
        self._write_byte(self.MODE1, 0xA1)
        time.sleep(0.1)
    
    def _write_byte(self, register, value):
        """Write a byte to a register"""
        try:
            self.bus.write_byte_data(self.address, register, value)
        except Exception as e:
            print(f"Error writing to register 0x{register:02X}: {e}")
            raise
    
    def _read_byte(self, register):
        """Read a byte from a register"""
        try:
            return self.bus.read_byte_data(self.address, register)
        except Exception as e:
            print(f"Error reading from register 0x{register:02X}: {e}")
            raise
    
    def set_pwm_frequency(self, frequency):
        """Set PWM frequency in Hz"""
        prescale = int(round(25000000.0 / (4096.0 * frequency)) - 1)
        
        # Read current mode
        old_mode = self._read_byte(self.MODE1)
        
        # Set sleep mode
        new_mode = (old_mode & 0x7F) | 0x10
        self._write_byte(self.MODE1, new_mode)
        
        # Set prescale
        self._write_byte(self.PRESCALE, prescale)
        
        # Restore mode
        self._write_byte(self.MODE1, old_mode)
        time.sleep(0.1)
        
        # Enable auto-increment
        self._write_byte(self.MODE1, old_mode | 0xA1)
        
        self.frequency = frequency
    
    def set_pwm(self, channel, on, off):
        """
        Set PWM output on a specific channel
        
        Args:
            channel (int): Channel number (0-15)
            on (int): Turn on time (0-4095)
            off (int): Turn off time (0-4095)
        """
        if channel < 0 or channel > 15:
            raise ValueError("Channel must be between 0 and 15")
        
        if on < 0 or on > 4095 or off < 0 or off > 4095:
            raise ValueError("PWM values must be between 0 and 4095")
        
        # Calculate register addresses
        on_l = channel * 4 + self.LED0_ON_L
        on_h = channel * 4 + self.LED0_ON_H
        off_l = channel * 4 + self.LED0_OFF_L
        off_h = channel * 4 + self.LED0_OFF_H
        
        # Write PWM values
        self._write_byte(on_l, on & 0xFF)
        self._write_byte(on_h, (on >> 8) & 0xFF)
        self._write_byte(off_l, off & 0xFF)
        self._write_byte(off_h, (off >> 8) & 0xFF)
    
    def set_servo_angle(self, channel, angle, min_pulse=500, max_pulse=2500):
        """
        Set servo angle on a specific channel
        
        Args:
            channel (int): Channel number (0-15)
            angle (float): Angle in degrees (0-180)
            min_pulse (int): Minimum pulse width in microseconds
            max_pulse (int): Maximum pulse width in microseconds
        """
        if angle < 0 or angle > 180:
            raise ValueError("Angle must be between 0 and 180 degrees")
        
        # Convert angle to pulse width
        pulse_width = min_pulse + (angle / 180.0) * (max_pulse - min_pulse)
        
        # Convert pulse width to PWM value
        pwm_value = int((pulse_width / 1000000.0) * self.frequency * 4096)
        
        # Set PWM output
        self.set_pwm(channel, 0, pwm_value)
    
    def set_servo_pulse(self, channel, pulse_width):
        """
        Set servo pulse width directly
        
        Args:
            channel (int): Channel number (0-15)
            pulse_width (int): Pulse width in microseconds
        """
        # Convert pulse width to PWM value
        pwm_value = int((pulse_width / 1000000.0) * self.frequency * 4096)
        
        # Set PWM output
        self.set_pwm(channel, 0, pwm_value)
    
    def set_all_servos(self, angles, min_pulse=500, max_pulse=2500):
        """
        Set multiple servos at once
        
        Args:
            angles (list): List of angles for each channel
            min_pulse (int): Minimum pulse width in microseconds
            max_pulse (int): Maximum pulse width in microseconds
        """
        for channel, angle in enumerate(angles):
            if channel < 16:  # PCA9685 has 16 channels
                self.set_servo_angle(channel, angle, min_pulse, max_pulse)
    
    def stop_servo(self, channel):
        """Stop servo on a specific channel"""
        self.set_pwm(channel, 0, 0)
    
    def stop_all_servos(self):
        """Stop all servos"""
        for channel in range(16):
            self.stop_servo(channel)
    
    def close(self):
        """Close I2C bus connection"""
        if self.bus:
            self.bus.close()

def main():
    """Main function demonstrating servo control"""
    print("PCA9685 Servo Control Demo")
    print("=" * 30)
    
    try:
        # Initialize PCA9685 controller
        print("Initializing PCA9685 controller...")
        controller = PCA9685(bus_number=1, address=0x40)
        print("Controller initialized successfully!")
        
        # Test servo on channel 0
        channel = 0
        print(f"\nTesting servo on channel {channel}")
        
        # Move servo to center position
        print("Moving servo to center (90 degrees)...")
        controller.set_servo_angle(channel, 90)
        time.sleep(2)
        
        # Move servo to minimum position
        print("Moving servo to minimum (0 degrees)...")
        controller.set_servo_angle(channel, 0)
        time.sleep(2)
        
        # Move servo to maximum position
        print("Moving servo to maximum (180 degrees)...")
        controller.set_servo_angle(channel, 180)
        time.sleep(2)
        
        # Return to center
        print("Returning servo to center...")
        controller.set_servo_angle(channel, 90)
        time.sleep(1)
        
        # Sweep servo back and forth
        print("\nPerforming servo sweep...")
        for angle in range(0, 181, 10):
            controller.set_servo_angle(channel, angle)
            time.sleep(0.1)
        
        for angle in range(180, -1, -10):
            controller.set_servo_angle(channel, angle)
            time.sleep(0.1)
        
        # Return to center
        controller.set_servo_angle(channel, 90)
        
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

