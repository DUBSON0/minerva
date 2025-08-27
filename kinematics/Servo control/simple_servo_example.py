#!/usr/bin/env python3
"""
Simple Servo Control Example
Basic example of controlling a single servo with PCA9685
"""

from servo_control import PCA9685
import time

def main():
    """Simple servo control example"""
    print("Simple Servo Control Example")
    print("=" * 30)
    
    try:
        # Initialize PCA9685 controller
        print("Initializing PCA9685 controller...")
        controller = PCA9685(bus_number=1, address=0x40)
        print("Controller initialized successfully!")
        
        # Control servo on channel 0
        channel = 0
        print(f"\nControlling servo on channel {channel}")
        
        # Move to center
        print("Moving to center (90 degrees)...")
        controller.set_servo_angle(channel, 90)
        time.sleep(1)
        
        # Move to left
        print("Moving to left (0 degrees)...")
        controller.set_servo_angle(channel, 0)
        time.sleep(1)
        
        # Move to right
        print("Moving to right (180 degrees)...")
        controller.set_servo_angle(channel, 180)
        time.sleep(1)
        
        # Return to center
        print("Returning to center...")
        controller.set_servo_angle(channel, 90)
        time.sleep(1)
        
        print("\nExample completed!")
        
    except KeyboardInterrupt:
        print("\nExample interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        if 'controller' in locals():
            controller.stop_all_servos()
            controller.close()
        print("Controller cleaned up")

if __name__ == "__main__":
    main()

