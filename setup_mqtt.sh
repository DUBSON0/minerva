#!/bin/bash

# Setup MQTT Server and Dependencies for PCA9685 Servo Controller
# Run this script on the Raspberry Pi

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${GREEN}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

# Check if running as root for system packages
check_sudo() {
    if [[ $EUID -eq 0 ]]; then
        print_warning "Running as root. This is fine for system setup."
    else
        print_status "Running as user. Will use sudo for system packages."
    fi
}

# Update system packages
update_system() {
    print_status "Updating system packages..."
    if ! sudo -n apt update 2>/dev/null; then
        print_warning "Cannot run sudo without password prompt. Skipping system update."
        print_warning "Please run 'sudo apt update && sudo apt upgrade -y' manually on the Pi"
        return 0
    fi
    sudo apt upgrade -y
    print_success "System packages updated"
}

# Install MQTT broker (Mosquitto)
install_mosquitto() {
    print_status "Installing Mosquitto MQTT broker..."
    
    # Check if we can run sudo without password
    if ! sudo -n true 2>/dev/null; then
        print_error "Cannot run sudo without password prompt."
        print_error "Please run these commands manually on the Pi:"
        echo "  sudo apt install -y mosquitto mosquitto-clients"
        echo "  sudo systemctl enable mosquitto"
        echo "  sudo systemctl start mosquitto"
        return 1
    fi
    
    # Install mosquitto and clients
    sudo apt install -y mosquitto mosquitto-clients
    
    # Enable and start mosquitto service
    sudo systemctl enable mosquitto
    sudo systemctl start mosquitto
    
    # Check if service is running
    if sudo systemctl is-active --quiet mosquitto; then
        print_success "Mosquitto MQTT broker installed and running"
    else
        print_error "Failed to start Mosquitto service"
        return 1
    fi
    
    # Test broker
    print_status "Testing MQTT broker..."
    timeout 5 mosquitto_pub -h localhost -t test/topic -m "test message" || {
        print_warning "MQTT test publish failed, but broker might still be working"
    }
    
    print_success "MQTT broker setup completed"
}

# Install development libraries
install_dev_libraries() {
    print_status "Installing development libraries..."
    
    # Install required libraries for C development
    sudo apt install -y \
        build-essential \
        cmake \
        pkg-config \
        libmosquitto-dev \
        libjson-c-dev \
        i2c-tools \
        libi2c-dev
    
    print_success "Development libraries installed"
}

# Install Python MQTT client
install_python_mqtt() {
    print_status "Installing Python MQTT client libraries..."
    
    # Install pip if not present
    sudo apt install -y python3-pip
    
    # Install paho-mqtt
    pip3 install paho-mqtt
    
    print_success "Python MQTT libraries installed"
}

# Configure Mosquitto
configure_mosquitto() {
    print_status "Configuring Mosquitto MQTT broker..."
    
    # Create basic configuration
    sudo tee /etc/mosquitto/conf.d/custom.conf > /dev/null << EOF
# Custom configuration for PCA9685 Servo Controller
listener 1883
allow_anonymous true
persistence true
persistence_location /var/lib/mosquitto/
log_dest file /var/log/mosquitto/mosquitto.log
log_type all
connection_messages true
log_timestamp true
EOF
    
    # Restart mosquitto to apply configuration
    sudo systemctl restart mosquitto
    
    # Check if still running
    if sudo systemctl is-active --quiet mosquitto; then
        print_success "Mosquitto configuration applied successfully"
    else
        print_error "Failed to restart Mosquitto with new configuration"
        return 1
    fi
}

# Test MQTT functionality
test_mqtt() {
    print_status "Testing MQTT functionality..."
    
    # Test publish/subscribe
    print_status "Testing publish/subscribe..."
    
    # Start subscriber in background
    timeout 10 mosquitto_sub -h localhost -t servo/test &
    SUB_PID=$!
    
    sleep 1
    
    # Publish test message
    mosquitto_pub -h localhost -t servo/test -m '{"test": "message", "timestamp": '$(date +%s)'}'
    
    # Wait for subscriber
    wait $SUB_PID 2>/dev/null || true
    
    print_success "MQTT test completed"
}

# Show firewall configuration
configure_firewall() {
    print_status "Checking firewall configuration..."
    
    # Check if ufw is installed and active
    if command -v ufw &> /dev/null; then
        if sudo ufw status | grep -q "Status: active"; then
            print_warning "UFW firewall is active"
            print_status "Opening MQTT port 1883..."
            sudo ufw allow 1883/tcp
            print_success "MQTT port 1883 opened in firewall"
        else
            print_status "UFW firewall is not active"
        fi
    else
        print_status "UFW firewall not installed"
    fi
}

# Display connection information
show_connection_info() {
    print_header "MQTT Broker Connection Information"
    
    # Get IP addresses
    IP_ADDRESSES=$(hostname -I)
    
    echo "MQTT Broker is running on:"
    for ip in $IP_ADDRESSES; do
        echo "  📡 mqtt://$ip:1883"
    done
    echo ""
    echo "Test the broker with:"
    echo "  📤 Publish: mosquitto_pub -h <ip> -t servo/control -m '{\"channel\": 0, \"angle\": 90}'"
    echo "  📥 Subscribe: mosquitto_sub -h <ip> -t servo/status"
    echo ""
    echo "Python test client:"
    echo "  🐍 python3 mqtt_servo_test.py --host <ip> --interactive"
    echo ""
}

# Main setup function
main() {
    print_header "PCA9685 MQTT Servo Controller Setup"
    
    check_sudo
    update_system
    install_dev_libraries
    install_mosquitto
    configure_mosquitto
    install_python_mqtt
    configure_firewall
    test_mqtt
    
    print_success "MQTT setup completed successfully!"
    show_connection_info
}

# Handle script arguments
case "${1:-install}" in
    "install"|"setup")
        main
        ;;
    "test")
        test_mqtt
        ;;
    "info")
        show_connection_info
        ;;
    "restart")
        print_status "Restarting Mosquitto service..."
        sudo systemctl restart mosquitto
        print_success "Mosquitto restarted"
        ;;
    "status")
        print_status "Checking Mosquitto status..."
        sudo systemctl status mosquitto --no-pager
        ;;
    "help"|*)
        echo "Usage: $0 [install|test|info|restart|status|help]"
        echo ""
        echo "Commands:"
        echo "  install  - Install and configure MQTT broker (default)"
        echo "  test     - Test MQTT functionality"
        echo "  info     - Show connection information"
        echo "  restart  - Restart Mosquitto service"
        echo "  status   - Show Mosquitto service status"
        echo "  help     - Show this help message"
        ;;
esac
