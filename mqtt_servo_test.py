#!/usr/bin/env python3
"""
MQTT Servo Test Client
Sends servo control commands to the PCA9685 MQTT driver
"""

import paho.mqtt.client as mqtt
import json
import time
import argparse
import sys

# MQTT Configuration
MQTT_HOST = "192.168.1.195"  # Pi IP address
MQTT_PORT = 1883
MQTT_TOPIC_SERVO = "servo/control"
MQTT_TOPIC_STATUS = "servo/status"

class ServoMQTTController:
    def __init__(self, host=MQTT_HOST, port=MQTT_PORT):
        self.host = host
        self.port = port
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        self.connected = False
        
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"✅ Connected to MQTT broker at {self.host}:{self.port}")
            self.connected = True
            # Subscribe to status messages
            client.subscribe(MQTT_TOPIC_STATUS)
            print(f"📡 Subscribed to status topic: {MQTT_TOPIC_STATUS}")
        else:
            print(f"❌ Failed to connect to MQTT broker. Return code: {rc}")
            
    def on_message(self, client, userdata, msg):
        try:
            status = json.loads(msg.payload.decode())
            if status.get('channel', -1) >= 0:
                print(f"📊 Status - Channel {status['channel']}: "
                      f"{status['duty_cycle']:.1f}% duty "
                      f"({status['angle']:.1f}°) - {status['status']}")
            else:
                print(f"📊 System Status: {status['status']}")
        except json.JSONDecodeError:
            print(f"📊 Raw status: {msg.payload.decode()}")
            
    def on_disconnect(self, client, userdata, rc):
        print("📡 Disconnected from MQTT broker")
        self.connected = False
        
    def connect(self):
        """Connect to MQTT broker"""
        try:
            print(f"🔗 Connecting to MQTT broker at {self.host}:{self.port}...")
            self.client.connect(self.host, self.port, 60)
            self.client.loop_start()
            
            # Wait for connection
            timeout = 5
            while not self.connected and timeout > 0:
                time.sleep(0.1)
                timeout -= 0.1
                
            return self.connected
        except Exception as e:
            print(f"❌ Connection error: {e}")
            return False
            
    def disconnect(self):
        """Disconnect from MQTT broker"""
        self.client.loop_stop()
        self.client.disconnect()
        
    def set_servo_duty_cycle(self, channel, duty_cycle):
        """Set servo duty cycle"""
        if not self.connected:
            print("❌ Not connected to MQTT broker")
            return False
            
        message = {
            "channel": channel,
            "duty_cycle": duty_cycle,
            "timestamp": int(time.time())
        }
        
        payload = json.dumps(message)
        result = self.client.publish(MQTT_TOPIC_SERVO, payload)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"📤 Sent: Channel {channel} → {duty_cycle}% duty cycle")
            return True
        else:
            print(f"❌ Failed to send message. Return code: {result.rc}")
            return False
            
    def set_servo_angle(self, channel, angle):
        """Set servo angle (0-180 degrees)"""
        if not self.connected:
            print("❌ Not connected to MQTT broker")
            return False
            
        message = {
            "channel": channel,
            "angle": angle,
            "timestamp": int(time.time())
        }
        
        payload = json.dumps(message)
        result = self.client.publish(MQTT_TOPIC_SERVO, payload)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"📤 Sent: Channel {channel} → {angle}° angle")
            return True
        else:
            print(f"❌ Failed to send message. Return code: {result.rc}")
            return False

def demo_sweep(controller, channel=0):
    """Demonstrate servo sweep from 0 to 180 degrees"""
    print(f"\n🔄 Starting servo sweep demo on channel {channel}")
    print("Moving from 0° to 180° and back...")
    
    # Sweep forward
    for angle in range(0, 181, 15):
        controller.set_servo_angle(channel, angle)
        time.sleep(0.5)
        
    time.sleep(1)
    
    # Sweep backward
    for angle in range(180, -1, -15):
        controller.set_servo_angle(channel, angle)
        time.sleep(0.5)
        
    # Return to center
    controller.set_servo_angle(channel, 90)
    print("✅ Sweep demo completed")

def interactive_mode(controller):
    """Interactive mode for manual servo control"""
    print("\n🎮 Interactive Mode")
    print("Commands:")
    print("  duty <channel> <duty_cycle>  - Set duty cycle (0-100%)")
    print("  angle <channel> <angle>      - Set angle (0-180°)")
    print("  sweep <channel>              - Run sweep demo")
    print("  center <channel>             - Move to center (90°)")
    print("  quit                         - Exit")
    print()
    
    while True:
        try:
            cmd = input("servo> ").strip().split()
            if not cmd:
                continue
                
            if cmd[0] == "quit":
                break
            elif cmd[0] == "duty" and len(cmd) == 3:
                channel = int(cmd[1])
                duty_cycle = float(cmd[2])
                controller.set_servo_duty_cycle(channel, duty_cycle)
            elif cmd[0] == "angle" and len(cmd) == 3:
                channel = int(cmd[1])
                angle = float(cmd[2])
                controller.set_servo_angle(channel, angle)
            elif cmd[0] == "sweep" and len(cmd) == 2:
                channel = int(cmd[1])
                demo_sweep(controller, channel)
            elif cmd[0] == "center" and len(cmd) == 2:
                channel = int(cmd[1])
                controller.set_servo_angle(channel, 90)
            else:
                print("❌ Invalid command. Type 'quit' to exit.")
                
        except KeyboardInterrupt:
            break
        except (ValueError, IndexError):
            print("❌ Invalid command format")
        except Exception as e:
            print(f"❌ Error: {e}")

def main():
    parser = argparse.ArgumentParser(description="MQTT Servo Controller Test Client")
    parser.add_argument("--host", default=MQTT_HOST, help="MQTT broker host")
    parser.add_argument("--port", type=int, default=MQTT_PORT, help="MQTT broker port")
    parser.add_argument("--channel", type=int, default=0, help="Servo channel (0-15)")
    parser.add_argument("--duty", type=float, help="Set duty cycle (0-100%%)")
    parser.add_argument("--angle", type=float, help="Set angle (0-180°)")
    parser.add_argument("--sweep", action="store_true", help="Run sweep demo")
    parser.add_argument("--interactive", "-i", action="store_true", help="Interactive mode")
    
    args = parser.parse_args()
    
    print("🤖 PCA9685 MQTT Servo Controller Test Client")
    print("=" * 50)
    
    # Create controller
    controller = ServoMQTTController(args.host, args.port)
    
    # Connect to broker
    if not controller.connect():
        print("❌ Failed to connect to MQTT broker")
        sys.exit(1)
        
    try:
        if args.interactive:
            interactive_mode(controller)
        elif args.sweep:
            demo_sweep(controller, args.channel)
        elif args.duty is not None:
            controller.set_servo_duty_cycle(args.channel, args.duty)
            time.sleep(2)  # Wait for status
        elif args.angle is not None:
            controller.set_servo_angle(args.channel, args.angle)
            time.sleep(2)  # Wait for status
        else:
            print("🎯 Quick test: Moving servo to center position")
            controller.set_servo_angle(args.channel, 90)
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    finally:
        controller.disconnect()
        print("👋 Disconnected from MQTT broker")

if __name__ == "__main__":
    main()
