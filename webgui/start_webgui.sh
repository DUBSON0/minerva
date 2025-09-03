#!/bin/bash

# PCA9685 Servo Web Controller Startup Script

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🤖 PCA9685 Servo Web Controller"
echo "==============================="

# Check Python version
if ! python3 --version >/dev/null 2>&1; then
    echo "❌ Error: Python 3 is not installed"
    exit 1
fi

echo "✅ Python version: $(python3 --version)"

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "🔄 Activating virtual environment..."
source venv/bin/activate

# Install dependencies
echo "📥 Installing dependencies..."
pip install --upgrade pip
pip install -r requirements.txt

# Check if MQTT broker is installed
echo "🔍 Checking MQTT broker..."
if ! command -v mosquitto >/dev/null 2>&1; then
    echo "📦 Installing MQTT broker (Mosquitto)..."
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        sudo apt update
        sudo apt install -y mosquitto mosquitto-clients
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        brew install mosquitto
    else
        echo "❌ Please install Mosquitto manually on your system"
        exit 1
    fi
fi

# Start MQTT broker in background
echo "🚀 Starting MQTT broker..."
sudo systemctl start mosquitto 2>/dev/null || {
    # Try to start mosquitto directly if systemctl fails
    mosquitto -d -p 1883 || {
        echo "❌ Failed to start MQTT broker"
        echo "Please make sure port 1883 is available"
        exit 1
    }
}

echo "✅ MQTT broker started"
echo "🚀 Starting web server..."
echo ""
echo "Web interface will be available at:"
echo "  Local:  http://localhost:5000"
echo "  Network: http://$(hostname -I | awk '{print $1}'):5000"
echo ""
echo "Make sure the PCA9685 MQTT driver is running on your Pi!"
echo "Press Ctrl+C to stop the server"
echo ""

# Start the Flask app
python3 app.py
