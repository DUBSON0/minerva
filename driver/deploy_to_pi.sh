#!/bin/bash

# Deploy and Run PCA9685 Servo Controller on Raspberry Pi
# Usage: ./deploy_to_pi.sh [option]
# Options: build, test, example, setup, clean, all

set -e  # Exit on any error

# Configuration
PI_USER="minerva"
PI_HOST="192.168.1.195"
PI_PASS="minerva"
PI_TARGET="$PI_USER@$PI_HOST"
REMOTE_DIR="/home/minerva"
# Determine script location and set file paths accordingly
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$SCRIPT_DIR" == *"/driver" ]]; then
    # Script is running from driver directory
    LOCAL_FILES="pca9685_mqtt_driver.c mqtt_servo_test.py setup_mqtt.sh"
else
    # Script is running via symlink from deployments directory
    LOCAL_FILES="$SCRIPT_DIR/pca9685_mqtt_driver.c $SCRIPT_DIR/mqtt_servo_test.py $SCRIPT_DIR/setup_mqtt.sh"
fi

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color

# Print functions
print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_header() {
    echo -e "${PURPLE}========================================${NC}"
    echo -e "${WHITE}$1${NC}"
    echo -e "${PURPLE}========================================${NC}"
}

# Function to check if sshpass is installed
check_sshpass() {
    if ! command -v sshpass &> /dev/null; then
        print_error "sshpass is required but not installed."
        echo "Install it with:"
        echo "  Ubuntu/Debian: sudo apt install sshpass"
        echo "  macOS: brew install sshpass"
        echo "  Or configure SSH key authentication instead"
        exit 1
    fi
}

# Function to test SSH connection
test_connection() {
    print_status "Testing SSH connection to $PI_TARGET..."
    if sshpass -p "$PI_PASS" ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=no "$PI_TARGET" "echo 'Connection successful'" &>/dev/null; then
        print_success "SSH connection to Raspberry Pi established"
        return 0
    else
        print_error "Failed to connect to Raspberry Pi"
        echo "Please check:"
        echo "  - IP address: $PI_HOST"
        echo "  - Username: $PI_USER"
        echo "  - Password: $PI_PASS"
        echo "  - Network connectivity"
        echo "  - SSH service running on Pi"
        return 1
    fi
}

# Function to create remote directory
create_remote_dir() {
    print_status "Creating remote directory: $REMOTE_DIR"
    sshpass -p "$PI_PASS" ssh -o StrictHostKeyChecking=no "$PI_TARGET" "mkdir -p $REMOTE_DIR"
}

# Function to transfer files
transfer_files() {
    print_status "Transferring files to Raspberry Pi..."
    
    # Check if all local files exist
    for file in $LOCAL_FILES; do
        if [[ ! -f "$file" ]]; then
            print_error "Local file not found: $file"
            exit 1
        fi
    done
    
    # Transfer files using scp
    for file in $LOCAL_FILES; do
        print_status "Transferring $file..."
        sshpass -p "$PI_PASS" scp -o StrictHostKeyChecking=no "$file" "$PI_TARGET:$REMOTE_DIR/"
    done
    
    print_success "All files transferred successfully"
}

# Function to run remote command
run_remote() {
    local cmd="$1"
    local description="$2"
    
    print_status "$description"
    sshpass -p "$PI_PASS" ssh -o StrictHostKeyChecking=no "$PI_TARGET" "cd $REMOTE_DIR && $cmd"
}

# Function to setup I2C on remote Pi
setup_i2c() {
    print_status "Setting up I2C on Raspberry Pi..."
    run_remote "sudo raspi-config nonint do_i2c 0" "Enabling I2C interface"
    run_remote "sudo usermod -a -G i2c \$USER" "Adding user to i2c group"
    print_warning "I2C setup complete. The Pi needs to be rebooted."
    print_warning "Run: sudo reboot on the Pi, then run this script again with 'build' option"
}

# Function to check I2C devices
check_i2c() {
    print_status "Checking I2C devices on Raspberry Pi..."
    run_remote "i2cdetect -y 1" "Scanning for I2C devices"
}

# Function to build on remote Pi
build_remote() {
    print_status "Building PCA9685 driver controller on Raspberry Pi..."
    run_remote "gcc -o pca9685_driver pca9685_driver.c" "Compiling PCA9685 driver program"
    print_success "Build completed successfully"
}

# Function to build MQTT version on remote Pi
build_mqtt_remote() {
    print_status "Building PCA9685 MQTT driver controller on Raspberry Pi..."
    run_remote "gcc -o pca9685_mqtt_driver pca9685_mqtt_driver.c -lmosquitto -ljson-c" "Compiling PCA9685 MQTT driver program"
    print_success "MQTT build completed successfully"
}

# Function to setup MQTT broker on Pi
setup_mqtt_remote() {
    print_status "Setting up MQTT broker on Raspberry Pi..."
    run_remote "chmod +x setup_mqtt.sh" "Making setup script executable"
    
    print_status "Installing MQTT broker and dependencies..."
    print_warning "This may prompt for sudo password on the Pi"
    
    # Try to install mosquitto directly
    if run_remote "sudo apt update && sudo apt install -y mosquitto mosquitto-clients libmosquitto-dev libjson-c-dev python3-pip" "Installing MQTT packages"; then
        print_success "MQTT packages installed"
    else
        print_error "Failed to install MQTT packages. You may need to run manually:"
        echo "  ssh minerva@192.168.1.195"
        echo "  sudo apt update"
        echo "  sudo apt install -y mosquitto mosquitto-clients libmosquitto-dev libjson-c-dev python3-pip"
        return 1
    fi
    
    # Enable and start mosquitto
    run_remote "sudo systemctl enable mosquitto && sudo systemctl start mosquitto" "Starting MQTT broker"
    
    # Install Python MQTT library using system package manager
    run_remote "sudo apt install -y python3-paho-mqtt" "Installing Python MQTT library"
    
    print_success "MQTT setup completed successfully"
}

# Function to test MQTT functionality
test_mqtt_remote() {
    print_status "Testing MQTT functionality on Raspberry Pi..."
    run_remote "./setup_mqtt.sh test" "Testing MQTT broker"
    print_success "MQTT test completed"
}

# Function to run PCA9685 driver program
run_test() {
    print_status "Running PCA9685 driver program on Raspberry Pi..."
    print_warning "This will start infinite loop mode. Use Ctrl+C to exit."
    run_remote "./pca9685_driver" "Executing PCA9685 driver program"
}

# Function to run MQTT PCA9685 driver program
run_mqtt_test() {
    print_status "Running PCA9685 MQTT driver program on Raspberry Pi..."
    print_warning "This will start MQTT servo controller. Use Ctrl+C to exit."
    run_remote "./pca9685_mqtt_driver" "Executing PCA9685 MQTT driver program"
}

# Function to run test (alias for run_test)
run_test_alias() {
    run_test
}

# Function to run full example (alias for run_test)
run_example() {
    run_test
}

# Function to clean remote build
clean_remote() {
    print_status "Cleaning build files on Raspberry Pi..."
    run_remote "rm -f pca9685_driver pca9685_mqtt_driver" "Cleaning build files"
    print_success "Clean completed"
}

# Function to show remote status
show_status() {
    print_status "Checking system status on Raspberry Pi..."
    run_remote "uname -a" "System information"
    run_remote "groups \$USER" "User groups"
    run_remote "ls -la /dev/i2c*" "I2C devices"
    run_remote "lsmod | grep i2c" "I2C modules"
}

# Main deployment function
deploy_all() {
    transfer_files
    build_remote
    check_i2c
    print_success "Deployment completed successfully!"
    echo ""
    echo "Next steps:"
    echo "  ./deploy_to_pi.sh test     - Run PCA9685 driver program"
    echo "  ./deploy_to_pi.sh check    - Check I2C devices"
    echo "  ./deploy_to_pi.sh status   - Check system status"
}

# Help function
show_help() {
    echo "Deploy PCA9685 Driver Controller to Raspberry Pi"
    echo "================================================"
    echo ""
    echo "Usage: $0 [option]"
    echo ""
    echo "Options:"
    echo "  all        - Transfer files, build, and check I2C (default)"
    echo "  transfer   - Transfer files only"
    echo "  build      - Build on remote Pi (after transfer)"
    echo "  test       - Run PCA9685 driver program (infinite loop)"
    echo "  driver     - Run PCA9685 driver program (alias for test)"
    echo "  example    - Run PCA9685 driver program (alias for test)"
    echo "  mqtt-setup - Setup MQTT broker and dependencies"
    echo "  mqtt-build - Build MQTT version of driver"
    echo "  mqtt-test  - Test MQTT functionality"
    echo "  mqtt-run   - Run MQTT servo controller"
    echo "  mqtt-all   - Complete MQTT setup and run"
    echo "  setup      - Setup I2C on Raspberry Pi (run once)"
    echo "  check      - Check I2C devices"
    echo "  clean      - Clean build files on remote Pi"
    echo "  status     - Show system status"
    echo "  help       - Show this help message"
    echo ""
    echo "Configuration:"
    echo "  Pi Address: $PI_HOST"
    echo "  Pi User:    $PI_USER"
    echo "  Remote Dir: $REMOTE_DIR"
    echo ""
    echo "Requirements:"
    echo "  - sshpass installed locally"
    echo "  - SSH access to Raspberry Pi"
    echo "  - I2C enabled on Pi (use 'setup' option)"
}

# Main script logic
main() {
    local action="${1:-all}"
    
    echo "========================================"
    echo "PCA9685 Driver Controller Deployment"
    echo "========================================"
    echo "Target: $PI_TARGET"
    echo "Action: $action"
    echo "========================================"
    
    # Check prerequisites
    check_sshpass
    
    # Test connection first
    if ! test_connection; then
        exit 1
    fi
    
    # Create remote directory
    create_remote_dir
    
    # Execute requested action
    case "$action" in
        "all")
            deploy_all
            ;;
        "transfer")
            transfer_files
            ;;
        "build")
            build_remote
            ;;
        "test")
            transfer_files
            build_remote
            run_test
            ;;
        "driver")
            transfer_files
            build_remote
            run_test_alias
            ;;
        "example")
            transfer_files
            build_remote
            run_example
            ;;
        "mqtt-setup")
            transfer_files
            setup_mqtt_remote
            ;;
        "mqtt-build")
            transfer_files
            build_mqtt_remote
            ;;
        "mqtt-test")
            test_mqtt_remote
            ;;
        "mqtt-run")
            run_mqtt_test
            ;;
        "mqtt-all")
            transfer_files
            setup_mqtt_remote
            build_mqtt_remote
            test_mqtt_remote
            print_success "MQTT setup completed! Ready to run servo controller."
            echo ""
            echo "To start the MQTT servo controller:"
            echo "  ./deploy_to_pi.sh mqtt-run"
            echo ""
            echo "To test with Python client:"
            echo "  python3 mqtt_servo_test.py --host $PI_HOST --interactive"
            ;;
        "setup")
            transfer_files
            setup_i2c
            ;;
        "check")
            check_i2c
            ;;
        "clean")
            clean_remote
            ;;
        "status")
            show_status
            ;;
        "help"|"-h"|"--help")
            show_help
            ;;
        *)
            print_error "Unknown option: $action"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Run main function with all arguments
main "$@"
