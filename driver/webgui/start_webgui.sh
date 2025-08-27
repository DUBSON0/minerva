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
