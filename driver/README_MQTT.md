# PCA9685 MQTT Servo Controller

This project provides MQTT control for PCA9685-based servo motors on Raspberry Pi.

## 🚀 Quick Start

### 1. Setup Everything at Once
```bash
./deploy_to_pi.sh mqtt-all
```

### 2. Start the MQTT Servo Controller
```bash
./deploy_to_pi.sh mqtt-run
```

### 3. Test with Python Client
```bash
python3 mqtt_servo_test.py --host 192.168.1.195 --interactive
```

## 📋 Step-by-Step Setup

### Prerequisites
- Raspberry Pi with Ubuntu/Raspbian
- PCA9685 PWM controller connected via I2C
- Network connection between your computer and Pi

### 1. Transfer Files
```bash
./deploy_to_pi.sh transfer
```

### 2. Setup MQTT Broker
```bash
./deploy_to_pi.sh mqtt-setup
```

### 3. Build MQTT Driver
```bash
./deploy_to_pi.sh mqtt-build
```

### 4. Test MQTT Functionality
```bash
./deploy_to_pi.sh mqtt-test
```

## 🎮 Control Your Servos

### MQTT Topics

- **Control Topic**: `servo/control`
- **Status Topic**: `servo/status`

### Message Format

#### Set Servo by Duty Cycle
```json
{
  "channel": 0,
  "duty_cycle": 7.5,
  "timestamp": 1692896400
}
```

#### Set Servo by Angle
```json
{
  "channel": 0,
  "angle": 90,
  "timestamp": 1692896400
}
```

### Python Test Client

#### Interactive Mode
```bash
python3 mqtt_servo_test.py --host 192.168.1.195 --interactive
```

Commands in interactive mode:
- `duty 0 7.5` - Set channel 0 to 7.5% duty cycle
- `angle 0 90` - Set channel 0 to 90 degrees
- `sweep 0` - Run sweep demo on channel 0
- `center 0` - Move channel 0 to center (90°)
- `quit` - Exit

#### Command Line Usage
```bash
# Set specific angle
python3 mqtt_servo_test.py --host 192.168.1.195 --channel 0 --angle 90

# Set specific duty cycle
python3 mqtt_servo_test.py --host 192.168.1.195 --channel 0 --duty 7.5

# Run sweep demo
python3 mqtt_servo_test.py --host 192.168.1.195 --channel 0 --sweep
```

## 🔧 Manual MQTT Commands

### Using mosquitto_pub
```bash
# Set servo to 90 degrees
mosquitto_pub -h 192.168.1.195 -t servo/control -m '{"channel": 0, "angle": 90}'

# Set servo to 7.5% duty cycle
mosquitto_pub -h 192.168.1.195 -t servo/control -m '{"channel": 0, "duty_cycle": 7.5}'
```

### Using mosquitto_sub to monitor status
```bash
mosquitto_sub -h 192.168.1.195 -t servo/status
```

## 📊 Servo Specifications

### Standard Servo Mapping (50Hz)
| Angle | Pulse Width | Duty Cycle | Description |
|-------|-------------|------------|-------------|
| 0°    | 1.0ms       | 5.0%       | Minimum position |
| 90°   | 1.5ms       | 7.5%       | Center position |
| 180°  | 2.0ms       | 10.0%      | Maximum position |

### Channel Range
- **Channels**: 0-15 (16 total channels)
- **Duty Cycle**: 0.0-100.0%
- **Angle**: 0-180°

## 🛠️ Architecture

### Components
1. **PCA9685 MQTT Driver** (`pca9685_mqtt_driver.c`)
   - Connects to MQTT broker
   - Controls PCA9685 via I2C
   - Publishes status messages

2. **MQTT Broker** (Mosquitto)
   - Runs on Raspberry Pi
   - Handles message routing
   - Port: 1883

3. **Python Test Client** (`mqtt_servo_test.py`)
   - Interactive servo control
   - Command-line interface
   - Status monitoring

### Message Flow
```
Client → MQTT Broker → PCA9685 Driver → I2C → PCA9685 → Servo
       ←              ← Status Messages ←      ←       ←
```

## 🔍 Troubleshooting

### Check MQTT Broker Status
```bash
./deploy_to_pi.sh status
# or on Pi directly:
sudo systemctl status mosquitto
```

### Test MQTT Connectivity
```bash
# Test publish
mosquitto_pub -h 192.168.1.195 -t test/topic -m "hello"

# Test subscribe
mosquitto_sub -h 192.168.1.195 -t test/topic
```

### Check I2C Connection
```bash
./deploy_to_pi.sh check
# or on Pi directly:
i2cdetect -y 1
```

### View Logs
```bash
# MQTT broker logs
sudo journalctl -u mosquitto -f

# System logs
sudo journalctl -f
```

## 🔧 Configuration

### MQTT Broker Settings
Edit `/etc/mosquitto/conf.d/custom.conf` on the Pi:
```
listener 1883
allow_anonymous true
persistence true
log_dest file /var/log/mosquitto/mosquitto.log
log_type all
```

### Firewall (if enabled)
```bash
sudo ufw allow 1883/tcp
```

## 📝 Development

### Compile Locally (requires MQTT libraries)
```bash
gcc -o pca9685_mqtt_driver pca9685_mqtt_driver.c -lmosquitto -ljson-c
```

### Install Dependencies on Ubuntu/Debian
```bash
sudo apt install libmosquitto-dev libjson-c-dev
```

## 🔗 Integration Examples

### Home Assistant
```yaml
# configuration.yaml
mqtt:
  broker: 192.168.1.195
  port: 1883

# Example automation
automation:
  - alias: "Move servo on motion"
    trigger:
      platform: state
      entity_id: binary_sensor.motion
      to: 'on'
    action:
      service: mqtt.publish
      data:
        topic: "servo/control"
        payload: '{"channel": 0, "angle": 180}'
```

### Node-RED
Create MQTT nodes with:
- **Server**: 192.168.1.195:1883
- **Publish Topic**: servo/control
- **Subscribe Topic**: servo/status

## 📋 Command Reference

### Deploy Script Commands
```bash
./deploy_to_pi.sh mqtt-all     # Complete setup
./deploy_to_pi.sh mqtt-setup   # Setup MQTT broker only
./deploy_to_pi.sh mqtt-build   # Build MQTT driver only
./deploy_to_pi.sh mqtt-test    # Test MQTT functionality
./deploy_to_pi.sh mqtt-run     # Run MQTT servo controller
```

### Python Client Commands
```bash
./mqtt_servo_test.py --help                    # Show help
./mqtt_servo_test.py --host IP --interactive   # Interactive mode
./mqtt_servo_test.py --host IP --sweep         # Sweep demo
./mqtt_servo_test.py --host IP --angle 90      # Set angle
./mqtt_servo_test.py --host IP --duty 7.5      # Set duty cycle
```
