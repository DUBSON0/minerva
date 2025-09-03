#!/usr/bin/env python3
"""
Flask Web GUI for PCA9685 MQTT Servo Controller
Provides a web interface to control 6 servo motors
"""

from flask import Flask, render_template, request, jsonify, redirect, url_for
from flask_socketio import SocketIO, emit
import paho.mqtt.client as mqtt
import json
import time
import threading
import logging
from datetime import datetime
import numpy as np
import sys
import os

# Add kinematics directory to path for importing IK module
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'kinematics'))
from robotic_arm_ik import SixDOFArmIK

# Configuration
MQTT_HOST = "localhost"  # MQTT broker runs on same computer as web UI
MQTT_PORT = 1883
MQTT_TOPIC_SERVO = "servo/control"
MQTT_TOPIC_STATUS = "servo/status"

# Robot arm configuration (6-DOF) - Simple planar robot for testing
# Joint positions (meters) - each joint's pivot point
ROBOT_P_HOME = np.array([
    [0.0, 0.0, 0.0],   # Joint 0: base
    [0.0, 0.0, 0.0],   # Joint 1: first link
    [0.3, 0.0, 0.0],   # Joint 2: second link
    [0.6, 0.0, 0.0],   # Joint 3: third link
    [0.9, 0.0, 0.0],   # Joint 4: fourth link
    [1.2, 0.0, 0.0],   # Joint 5: fifth link
], dtype=float)

# Joint rotation axes (unit vectors) - all rotate around Z for planar motion
ROBOT_A_HOME = np.array([
    [0.0, 0.0, 1.0],   # Joint 0: base rotation
    [0.0, 0.0, 1.0],   # Joint 1: first joint rotation
    [0.0, 0.0, 1.0],   # Joint 2: second joint rotation
    [0.0, 0.0, 1.0],   # Joint 3: third joint rotation
    [0.0, 0.0, 1.0],   # Joint 4: fourth joint rotation
    [0.0, 0.0, 1.0],   # Joint 5: fifth joint rotation
], dtype=float)

# Tool offset (end-effector position relative to last joint)
TOOL_OFFSET = np.array([0.3, 0.0, 0.0])  # 30cm tool offset along X

app = Flask(__name__)
app.config['SECRET_KEY'] = 'servo_control_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Logging setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ServoController:
    def __init__(self):
        self.mqtt_client = None
        self.connected = False
        self.servo_states = {}
        self.last_status = {}

        # Initialize IK solver for the robot arm
        self.ik_solver = SixDOFArmIK(ROBOT_P_HOME, ROBOT_A_HOME, tool_offset=TOOL_OFFSET)

        # Initialize servo states for 6 motors (channels 0-5)
        for i in range(6):
            self.servo_states[i] = {
                'angle': 90.0,
                'duty_cycle': 7.5,
                'status': 'unknown',
                'last_update': None
            }

        self.setup_mqtt()
    
    def setup_mqtt(self):
        """Setup MQTT client"""
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            logger.info(f"Connecting to MQTT broker at {MQTT_HOST}:{MQTT_PORT}")
            self.mqtt_client.connect(MQTT_HOST, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            
        except Exception as e:
            logger.error(f"MQTT setup error: {e}")
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback for MQTT connection"""
        if rc == 0:
            self.connected = True
            logger.info("Connected to MQTT broker")
            client.subscribe(MQTT_TOPIC_STATUS)
            logger.info(f"Subscribed to {MQTT_TOPIC_STATUS}")
            
            # Notify web clients
            socketio.emit('mqtt_status', {'connected': True})
        else:
            logger.error(f"Failed to connect to MQTT broker: {rc}")
            socketio.emit('mqtt_status', {'connected': False})
    
    def on_mqtt_message(self, client, userdata, msg):
        """Callback for MQTT messages"""
        try:
            status = json.loads(msg.payload.decode())
            logger.info(f"Received status: {status}")
            
            # Update servo state if it's a valid channel
            channel = status.get('channel', -1)
            if 0 <= channel <= 5:
                self.servo_states[channel].update({
                    'angle': status.get('angle', self.servo_states[channel]['angle']),
                    'duty_cycle': status.get('duty_cycle', self.servo_states[channel]['duty_cycle']),
                    'status': status.get('status', 'unknown'),
                    'last_update': datetime.now().isoformat()
                })
                
                # Notify web clients
                socketio.emit('servo_status', {
                    'channel': channel,
                    'state': self.servo_states[channel]
                })
            
            # Store general status
            self.last_status = status
            
        except json.JSONDecodeError as e:
            logger.error(f"JSON decode error: {e}")
        except Exception as e:
            logger.error(f"Message processing error: {e}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback for MQTT disconnection"""
        self.connected = False
        logger.warning("Disconnected from MQTT broker")
        socketio.emit('mqtt_status', {'connected': False})
    
    def set_servo_angle(self, channel, angle):
        """Set servo angle"""
        if not self.connected:
            return False, "MQTT not connected"
        
        if not (0 <= channel <= 5):
            return False, "Invalid channel (must be 0-5)"
        
        if not (0 <= angle <= 180):
            return False, "Invalid angle (must be 0-180)"
        
        message = {
            "channel": channel,
            "angle": angle,
            "timestamp": int(time.time())
        }
        
        try:
            result = self.mqtt_client.publish(MQTT_TOPIC_SERVO, json.dumps(message))
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                logger.info(f"Sent command: Channel {channel} → {angle}°")
                return True, "Command sent"
            else:
                return False, f"MQTT publish failed: {result.rc}"
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            return False, str(e)
    
    def set_servo_duty_cycle(self, channel, duty_cycle):
        """Set servo duty cycle"""
        if not self.connected:
            return False, "MQTT not connected"
        
        if not (0 <= channel <= 5):
            return False, "Invalid channel (must be 0-5)"
        
        if not (0 <= duty_cycle <= 100):
            return False, "Invalid duty cycle (must be 0-100)"
        
        message = {
            "channel": channel,
            "duty_cycle": duty_cycle,
            "timestamp": int(time.time())
        }
        
        try:
            result = self.mqtt_client.publish(MQTT_TOPIC_SERVO, json.dumps(message))
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                logger.info(f"Sent command: Channel {channel} → {duty_cycle}% duty")
                return True, "Command sent"
            else:
                return False, f"MQTT publish failed: {result.rc}"
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            return False, str(e)
    
    def get_servo_states(self):
        """Get all servo states"""
        return self.servo_states
    
    def is_connected(self):
        """Check if MQTT is connected"""
        return self.connected

    def compute_ik(self, target_position, target_orientation=None):
        """Compute inverse kinematics for target position and orientation

        Args:
            target_position: [x, y, z] coordinates in meters
            target_orientation: quaternion [w, x, y, z] or None for default

        Returns:
            dict: {'success': bool, 'angles': list, 'message': str, 'position': list}
        """
        try:
            # Default orientation (no rotation) if not provided
            if target_orientation is None:
                target_q = np.array([1.0, 0.0, 0.0, 0.0])  # identity quaternion
            else:
                target_q = np.array(target_orientation)

            # Convert target position to numpy array
            target_p = np.array(target_position, dtype=float)

            # Solve IK
            solution = self.ik_solver.solve_ik(
                target_p,
                target_q,
                max_iters=200,
                pos_tol=1e-3,
                ori_tol=1e-2
            )

            if solution.success:
                # Convert angles from radians to degrees for servo control
                angles_deg = np.degrees(solution.angles)

                # Ensure angles are within servo range (0-180 degrees)
                angles_deg = np.clip(angles_deg, 0, 180)

                return {
                    'success': True,
                    'angles': angles_deg.tolist(),
                    'message': f'IK converged in {solution.iters} iterations',
                    'position': solution.fk.end_effector_position.tolist(),
                    'target_position': target_position
                }
            else:
                return {
                    'success': False,
                    'angles': None,
                    'message': solution.message,
                    'position': solution.fk.end_effector_position.tolist(),
                    'target_position': target_position
                }

        except Exception as e:
            logger.error(f"IK computation error: {e}")
            return {
                'success': False,
                'angles': None,
                'message': f"IK computation failed: {str(e)}",
                'position': None,
                'target_position': target_position
            }

    def move_to_ik_position(self, angles):
        """Move servos to angles computed by IK

        Args:
            angles: list of 6 angles in degrees

        Returns:
            dict: {'success': bool, 'results': list, 'message': str}
        """
        if len(angles) != 6:
            return {
                'success': False,
                'results': [],
                'message': 'Must provide exactly 6 angles'
            }

        results = []
        success_count = 0

        for i, angle in enumerate(angles):
            success, message = self.set_servo_angle(i, angle)
            results.append({
                'channel': i,
                'success': success,
                'message': message,
                'angle': angle
            })
            if success:
                success_count += 1

        overall_success = success_count == 6
        return {
            'success': overall_success,
            'results': results,
            'message': f'Moved {success_count}/6 joints successfully'
        }

# Global servo controller instance
servo_controller = ServoController()

@app.route('/')
def index():
    """Main page"""
    return render_template('index.html')

@app.route('/api/servos')
def get_servos():
    """Get all servo states"""
    return jsonify({
        'servos': servo_controller.get_servo_states(),
        'mqtt_connected': servo_controller.is_connected()
    })

@app.route('/api/servo/<int:channel>/angle', methods=['POST'])
def set_servo_angle(channel):
    """Set servo angle"""
    try:
        data = request.get_json()
        angle = float(data.get('angle', 90))
        
        success, message = servo_controller.set_servo_angle(channel, angle)
        return jsonify({
            'success': success,
            'message': message,
            'channel': channel,
            'angle': angle
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'message': str(e)
        }), 400

@app.route('/api/servo/<int:channel>/duty', methods=['POST'])
def set_servo_duty(channel):
    """Set servo duty cycle"""
    try:
        data = request.get_json()
        duty_cycle = float(data.get('duty_cycle', 7.5))
        
        success, message = servo_controller.set_servo_duty_cycle(channel, duty_cycle)
        return jsonify({
            'success': success,
            'message': message,
            'channel': channel,
            'duty_cycle': duty_cycle
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'message': str(e)
        }), 400

@app.route('/api/servo/<int:channel>/center', methods=['POST'])
def center_servo(channel):
    """Center servo (90 degrees)"""
    success, message = servo_controller.set_servo_angle(channel, 90)
    return jsonify({
        'success': success,
        'message': message,
        'channel': channel,
        'angle': 90
    })

@app.route('/api/servos/center', methods=['POST'])
def center_all_servos():
    """Center all servos"""
    results = []
    for channel in range(6):
        success, message = servo_controller.set_servo_angle(channel, 90)
        results.append({
            'channel': channel,
            'success': success,
            'message': message
        })
        time.sleep(0.1)  # Small delay between commands
    
    return jsonify({
        'results': results
    })

@socketio.on('connect')
def handle_connect():
    """Handle WebSocket connection"""
    logger.info('Client connected to WebSocket')
    emit('mqtt_status', {'connected': servo_controller.is_connected()})
    emit('servo_states', {'servos': servo_controller.get_servo_states()})

@socketio.on('disconnect')
def handle_disconnect():
    """Handle WebSocket disconnection"""
    logger.info('Client disconnected from WebSocket')

@socketio.on('set_servo_angle')
def handle_set_servo_angle(data):
    """Handle WebSocket servo angle command"""
    channel = int(data.get('channel', 0))
    angle = float(data.get('angle', 90))

    success, message = servo_controller.set_servo_angle(channel, angle)
    emit('command_result', {
        'success': success,
        'message': message,
        'channel': channel,
        'angle': angle
    })

# IK API Endpoints
@app.route('/api/ik/compute', methods=['POST'])
def compute_ik():
    """Compute inverse kinematics for target position"""
    try:
        data = request.get_json()

        # Extract position coordinates
        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        z = float(data.get('z', 0.0))
        target_position = [x, y, z]

        # Extract orientation if provided (optional)
        orientation = None
        if 'orientation' in data:
            ori_data = data['orientation']
            if isinstance(ori_data, list) and len(ori_data) == 4:
                orientation = ori_data

        result = servo_controller.compute_ik(target_position, orientation)

        return jsonify(result)

    except Exception as e:
        return jsonify({
            'success': False,
            'message': f"API error: {str(e)}",
            'angles': None,
            'position': None
        }), 400

@app.route('/api/ik/move', methods=['POST'])
def move_to_ik_position():
    """Move robot to IK-computed position"""
    try:
        data = request.get_json()
        angles = data.get('angles', [])

        if not isinstance(angles, list) or len(angles) != 6:
            return jsonify({
                'success': False,
                'message': 'Must provide exactly 6 angles'
            }), 400

        # Convert to float and validate
        angles = [float(angle) for angle in angles]

        result = servo_controller.move_to_ik_position(angles)

        return jsonify(result)

    except Exception as e:
        return jsonify({
            'success': False,
            'message': f"API error: {str(e)}"
        }), 400

@app.route('/api/ik/compute_and_move', methods=['POST'])
def compute_and_move():
    """Compute IK and immediately move to position"""
    try:
        data = request.get_json()

        # Extract position coordinates
        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        z = float(data.get('z', 0.0))
        target_position = [x, y, z]

        # Extract orientation if provided (optional)
        orientation = None
        if 'orientation' in data:
            ori_data = data['orientation']
            if isinstance(ori_data, list) and len(ori_data) == 4:
                orientation = ori_data

        # First compute IK
        ik_result = servo_controller.compute_ik(target_position, orientation)

        if not ik_result['success']:
            return jsonify({
                'success': False,
                'message': f"IK computation failed: {ik_result['message']}",
                'ik_result': ik_result
            }), 400

        # Then move to position
        move_result = servo_controller.move_to_ik_position(ik_result['angles'])

        return jsonify({
            'success': move_result['success'],
            'message': move_result['message'],
            'ik_result': ik_result,
            'move_result': move_result
        })

    except Exception as e:
        return jsonify({
            'success': False,
            'message': f"API error: {str(e)}"
        }), 400

if __name__ == '__main__':
    print("🤖 PCA9685 MQTT Servo Web Controller")
    print("=" * 50)
    print(f"MQTT Broker: {MQTT_HOST}:{MQTT_PORT}")
    print(f"Control Topic: {MQTT_TOPIC_SERVO}")
    print(f"Status Topic: {MQTT_TOPIC_STATUS}")
    print("Web GUI will be available at: http://localhost:5000")
    print("=" * 50)
    
    # Start Flask app with SocketIO
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, allow_unsafe_werkzeug=True)
