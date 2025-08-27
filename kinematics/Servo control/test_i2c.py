#!/usr/bin/env python3
"""
Simple I2C Test Script
Tests I2C connectivity and detects PCA9685 device
"""

import smbus2
import sys

def test_i2c_connection():
    """Test I2C connection and scan for devices"""
    print("I2C Connection Test")
    print("=" * 30)
    
    # Try different bus numbers
    bus_numbers = [0, 1]
    
    for bus_num in bus_numbers:
        print(f"\nTesting I2C bus {bus_num}...")
        
        try:
            bus = smbus2.SMBus(bus_num)
            print(f"✓ Successfully opened I2C bus {bus_num}")
            
            # Scan for devices
            print(f"Scanning I2C bus {bus_num} for devices...")
            devices_found = []
            
            for address in range(0x03, 0x78):
                try:
                    bus.read_byte_data(address, 0)
                    devices_found.append(address)
                    print(f"  Device found at address 0x{address:02X}")
                except:
                    pass
            
            if devices_found:
                print(f"\nFound {len(devices_found)} I2C device(s):")
                for addr in devices_found:
                    print(f"  - 0x{addr:02X}")
                    
                    # Check if it's a PCA9685
                    if addr == 0x40:
                        print("    ✓ This appears to be a PCA9685 device!")
                    elif addr == 0x41:
                        print("    ✓ This appears to be a PCA9685 device (alternate address)!")
                    else:
                        print("    ? Unknown device type")
            else:
                print("  No I2C devices found on this bus")
            
            bus.close()
            print(f"✓ Closed I2C bus {bus_num}")
            
        except Exception as e:
            print(f"✗ Failed to open I2C bus {bus_num}: {e}")
    
    print("\n" + "=" * 30)
    print("Test completed!")

def test_pca9685_communication():
    """Test direct communication with PCA9685 if found"""
    print("\nPCA9685 Communication Test")
    print("=" * 30)
    
    try:
        # Try bus 1 first (common on Raspberry Pi)
        bus = smbus2.SMBus(1)
        print("Testing communication with PCA9685 at address 0x40...")
        
        # Try to read MODE1 register
        try:
            mode1 = bus.read_byte_data(0x40, 0x00)
            print(f"✓ Successfully read MODE1 register: 0x{mode1:02X}")
            
            # Try to write to MODE1 register
            bus.write_byte_data(0x40, 0x00, mode1)
            print("✓ Successfully wrote to MODE1 register")
            
            print("✓ PCA9685 communication test passed!")
            
        except Exception as e:
            print(f"✗ PCA9685 communication test failed: {e}")
        
        bus.close()
        
    except Exception as e:
        print(f"✗ Could not test PCA9685 communication: {e}")

if __name__ == "__main__":
    try:
        test_i2c_connection()
        test_pca9685_communication()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed with error: {e}")
        sys.exit(1)

