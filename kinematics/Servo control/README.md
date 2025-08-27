# PCA9685 Servo Control Script

This Python script controls servo motors using the PCA9685 16-channel 12-bit PWM driver via I2C communication. It's designed for use with Raspberry Pi and Ubuntu systems.

## Features

- Control up to 16 servo motors independently
- Set servo angles (0-180 degrees) or pulse widths directly
- Configurable PWM frequency (default 50Hz for servos)
- Error handling and safe shutdown
- Support for different servo types with customizable pulse ranges

## Hardware Requirements

- PCA9685 16-channel PWM driver board
- Servo motors (standard 180-degree servos recommended)
- Raspberry Pi or Ubuntu system with I2C support
- Power supply for servos (typically 5V)
- Breadboard and jumper wires

## Wiring Diagram

```
Raspberry Pi/Ubuntu System    PCA9685 Board
┌─────────────────┐         ┌─────────────────┐
│                 │         │                 │
│ 3.3V ──────────┼─────────┤ VCC             │
│                 │         │                 │
│ GND ───────────┼─────────┤ GND             │
│                 │         │                 │
│ SDA (GPIO2) ───┼─────────┤ SDA             │
│                 │         │                 │
│ SCL (GPIO3) ───┼─────────┤ SCL             │
│                 │         │                 │
└─────────────────┘         └─────────────────┘
                                    │
                                    │
                            ┌───────┴───────┐
                            │               │
                            │ Servo Motors  │
                            │               │
                            │ Channel 0 ────┼─── Servo 1
                            │ Channel 1 ────┼─── Servo 2
                            │ Channel 2 ────┼─── Servo 3
                            │     ...       │     ...
                            │ Channel 15 ───┼─── Servo 16
                            │               │
                            └───────────────┘
```

## Installation

### 1. Enable I2C on Raspberry Pi

```bash
# Enable I2C interface
sudo raspi-config

# Navigate to: Interface Options → I2C → Enable
# Reboot after enabling
sudo reboot
```

### 2. Install Python Dependencies

```bash
# Install system dependencies
sudo apt-get update
sudo apt-get install python3-pip python3-dev

# Install Python package
pip3 install -r requirements.txt
```

### 3. Verify I2C Connection

```bash
# Check if I2C devices are detected
sudo i2cdetect -y 1

# You should see the PCA9685 at address 0x40 (or your configured address)
```

## Usage

### Basic Usage

```python
from servo_control import PCA9685

# Initialize controller
controller = PCA9685(bus_number=1, address=0x40)

# Move servo on channel 0 to 90 degrees
controller.set_servo_angle(0, 90)

# Move servo on channel 1 to 45 degrees
controller.set_servo_angle(1, 45)

# Clean up
controller.close()
```

### Advanced Usage

```python
# Set custom pulse width range for specific servo
controller.set_servo_angle(0, 90, min_pulse=600, max_pulse=2400)

# Set pulse width directly (in microseconds)
controller.set_servo_pulse(0, 1500)  # Center position

# Control multiple servos at once
angles = [90, 45, 135, 0]  # Channels 0, 1, 2, 3
controller.set_all_servos(angles)

# Stop specific servo
controller.stop_servo(0)

# Stop all servos
controller.stop_all_servos()
```

### Running the Demo

```bash
# Run the demonstration script
python3 servo_control.py
```

The demo will:
1. Initialize the PCA9685 controller
2. Test servo movement on channel 0
3. Perform a sweep motion
4. Clean up and stop all servos

## Configuration

### I2C Address

The default I2C address is 0x40. If you need to change it:

```python
controller = PCA9685(bus_number=1, address=0x41)  # Different address
```

### PWM Frequency

The default frequency is 50Hz, which is standard for most servos. To change:

```python
controller.set_pwm_frequency(60)  # 60Hz for some digital servos
```

### Servo Pulse Ranges

Different servo models may require different pulse ranges:

```python
# Standard servo (500-2500 μs)
controller.set_servo_angle(0, 90, min_pulse=500, max_pulse=2500)

# Digital servo (600-2400 μs)
controller.set_servo_angle(0, 90, min_pulse=600, max_pulse=2400)

# Custom range
controller.set_servo_angle(0, 90, min_pulse=800, max_pulse=2200)
```

## Troubleshooting

### Common Issues

1. **Permission Denied**: Run with sudo or add user to i2c group
   ```bash
   sudo usermod -a -G i2c $USER
   # Log out and back in
   ```

2. **I2C Device Not Found**: Check wiring and enable I2C interface
   ```bash
   sudo i2cdetect -y 1
   ```

3. **Servo Not Moving**: Check power supply and pulse range settings

4. **Jittery Movement**: Ensure stable power supply and check for interference

### Debug Mode

Enable verbose output by modifying the script:

```python
# Add debug prints in _write_byte and _read_byte methods
print(f"Writing 0x{value:02X} to register 0x{register:02X}")
```

## Safety Notes

- Always ensure proper power supply for servos
- Don't exceed servo voltage ratings
- Be careful with moving parts
- Stop servos before disconnecting power
- Use appropriate current limiting if needed

## License

This script is provided as-is for educational and development purposes.

## Contributing

Feel free to submit issues, feature requests, or pull requests to improve the script.

