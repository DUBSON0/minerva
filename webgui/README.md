# PCA9685 Servo Web Controller

A modern web interface for controlling 6 servo motors connected to a PCA9685 PWM controller via MQTT.

## Features

- 🎮 **Real-time Control**: Interactive web interface with sliders and precise input controls
- 📡 **MQTT Integration**: Communicates with the PCA9685 MQTT driver for reliable servo control  
- 🔄 **Live Updates**: WebSocket connection provides real-time status updates
- 📱 **Responsive Design**: Works on desktop, tablet, and mobile devices
- 🎯 **Individual & Group Control**: Control servos individually or center all at once
- 📊 **Status Monitoring**: Real-time feedback on servo positions and connection status

## Quick Start

### 1. Install Dependencies

```bash
cd webgui
pip3 install -r requirements.txt
```

### 2. Configure MQTT Connection

Edit the `MQTT_HOST` variable in `app.py` to match your Pi's IP address:

```python
MQTT_HOST = "192.168.1.195"  # Change to your Pi's IP
```

### 3. Start the Web Server

```bash
python3 app.py
```

### 4. Open in Browser

Navigate to: `http://localhost:5000`

Or from another device: `http://YOUR_COMPUTER_IP:5000`

## System Requirements

### On Your Computer (Web Server)
- Python 3.7+
- Flask and dependencies (see requirements.txt)
- Network connection to Raspberry Pi

### On Raspberry Pi (MQTT Driver)
- PCA9685 MQTT driver running (`pca9685_mqtt_driver`)
- MQTT broker (Mosquitto) 
- PCA9685 connected via I2C
- 6 servo motors on channels 0-5

## Web Interface Guide

### Status Bar
- **MQTT Status**: Shows connection to the MQTT broker
- **Last Update**: Timestamp of the most recent servo status update

### Global Controls
- **Center All**: Moves all 6 servos to 90° (center position)
- **Refresh Status**: Manually updates servo states from the MQTT driver
- **Clear Messages**: Clears the message log

### Individual Servo Controls
Each servo motor has its own control card with:

- **Angle Display**: Current servo angle (0-180°)
- **Duty Cycle Display**: Current PWM duty cycle percentage
- **Status Indicator**: Success/Error/Unknown status
- **Angle Slider**: Drag to set servo angle with real-time updates
- **Precise Input**: Enter exact angle values
- **Set Button**: Apply the precise angle value
- **Center Button**: Move individual servo to 90°

### Message Log
Real-time log of all commands sent and status updates received.

## API Reference

The web server provides REST API endpoints for integration:

### Get All Servo States
```http
GET /api/servos
```

### Set Servo Angle
```http
POST /api/servo/<channel>/angle
Content-Type: application/json

{
  "angle": 90
}
```

### Set Servo Duty Cycle
```http
POST /api/servo/<channel>/duty
Content-Type: application/json

{
  "duty_cycle": 7.5
}
```

### Center Single Servo
```http
POST /api/servo/<channel>/center
```

### Center All Servos
```http
POST /api/servos/center
```

## WebSocket Events

Real-time updates via Socket.IO:

### Client → Server
- `set_servo_angle`: Set servo angle
- `connect`/`disconnect`: Connection management

### Server → Client
- `mqtt_status`: MQTT connection status
- `servo_status`: Individual servo state updates
- `servo_states`: All servo states
- `command_result`: Command execution results

## MQTT Integration

The web server communicates with the PCA9685 driver via MQTT:

### Topics
- **Control**: `servo/control` - Send servo commands
- **Status**: `servo/status` - Receive servo status updates

### Message Format
```json
{
  "channel": 0,
  "angle": 90,
  "timestamp": 1692896400
}
```

## Troubleshooting

### Web Server Won't Start
- Check Python dependencies: `pip3 install -r requirements.txt`
- Verify port 5000 is available
- Check firewall settings

### MQTT Connection Failed
- Verify Pi IP address in `app.py`
- Ensure MQTT broker is running on Pi: `sudo systemctl status mosquitto`
- Test MQTT manually: `mosquitto_pub -h PI_IP -t test -m "hello"`

### Servos Not Responding
- Check PCA9685 MQTT driver is running on Pi
- Verify I2C connection: `i2cdetect -y 1`
- Check servo power supply
- Monitor MQTT status messages

### Network Issues
- Ensure both devices are on same network
- Check firewall settings (ports 5000, 1883)
- Test network connectivity: `ping PI_IP`

## Development

### Project Structure
```
webgui/
├── app.py              # Flask web server
├── templates/
│   └── index.html      # Web interface
├── requirements.txt    # Python dependencies
└── README.md          # This file
```

### Customization
- **Styling**: Edit CSS in `templates/index.html`
- **Motor Count**: Change servo loop ranges in both `app.py` and `index.html`
- **MQTT Topics**: Modify topic constants in `app.py`
- **Update Intervals**: Adjust WebSocket and polling rates

### Adding Features
The modular design makes it easy to add:
- Servo presets and sequences
- Configuration file support  
- User authentication
- Multiple servo controller support
- Data logging and analytics

## License

This project is part of the Minerva servo control system.
