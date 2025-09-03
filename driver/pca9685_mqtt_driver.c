#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <signal.h>
#include <time.h>
#include <mosquitto.h>
#include <json-c/json.h>

#define PCA9685_ADDR 0x40
#define MODE1_REG 0x00
#define PRESCALE_REG 0xFE
#define LED0_ON_L 0x06

// MQTT Configuration
#define MQTT_HOST "192.168.1.241"  // IP of computer running web UI
#define MQTT_PORT 1883
#define MQTT_KEEPALIVE 60
#define MQTT_TOPIC_SERVO "servo/control"
#define MQTT_TOPIC_STATUS "servo/status"
#define CLIENT_ID "pca9685_driver"

// Global file descriptor for I2C device
static int pca9685_driver_fd = -1;
static int pca9685_driver_initialized = 0;
static struct mosquitto *mosq = NULL;
static int running = 1;

// Servo calibration constants
#define SERVO_MIN_DUTY_CYCLE 5.0    // 1ms pulse width (0 degrees)
#define SERVO_MAX_DUTY_CYCLE 10.0   // 2ms pulse width (180 degrees)
#define SERVO_CENTER_DUTY_CYCLE 7.5 // 1.5ms pulse width (90 degrees)

/**
 * Signal handler for graceful shutdown
 */
void signal_handler(int sig) {
    printf("\nReceived signal %d, shutting down gracefully...\n", sig);
    running = 0;
}

/**
 * Initialize the PCA9685 PWM controller
 * Returns: 0 on success, -1 on error
 */
int pca9685_driver_init() {
    // Open I2C device
    pca9685_driver_fd = open("/dev/i2c-1", O_RDWR);
    if (pca9685_driver_fd < 0) {
        perror("Failed to open I2C device");
        return -1;
    }
    
    // Set I2C slave address
    if (ioctl(pca9685_driver_fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        perror("Failed to set I2C slave address");
        close(pca9685_driver_fd);
        return -1;
    }
    
    // Reset PCA9685
    unsigned char mode1_reset[2] = {MODE1_REG, 0x00};
    if (write(pca9685_driver_fd, mode1_reset, 2) != 2) {
        perror("Failed to reset PCA9685");
        close(pca9685_driver_fd);
        return -1;
    }
    
    // Set PWM frequency to 50Hz (20ms period) for servos
    // Formula: prescale = round(25MHz / (4096 * frequency)) - 1
    // For 50Hz: prescale = round(25000000 / (4096 * 50)) - 1 = 121
    unsigned char prescale_cmd[2] = {PRESCALE_REG, 121};
    if (write(pca9685_driver_fd, prescale_cmd, 2) != 2) {
        perror("Failed to set PWM frequency");
        close(pca9685_driver_fd);
        return -1;
    }
    
    // Enable oscillator
    unsigned char mode1_enable[2] = {MODE1_REG, 0x80};
    if (write(pca9685_driver_fd, mode1_enable, 2) != 2) {
        perror("Failed to enable oscillator");
        close(pca9685_driver_fd);
        return -1;
    }
    
    usleep(500); // Wait for oscillator to stabilize
    pca9685_driver_initialized = 1;
    printf("PCA9685 initialized successfully at 50Hz\n");
    return 0;
}

/**
 * Convert angle to duty cycle
 */
float angle_to_duty_cycle(float angle) {
    // Clamp angle to 0-180 range
    if (angle < 0.0) angle = 0.0;
    if (angle > 180.0) angle = 180.0;
    
    // Linear interpolation between min and max duty cycles
    return SERVO_MIN_DUTY_CYCLE + (angle / 180.0) * (SERVO_MAX_DUTY_CYCLE - SERVO_MIN_DUTY_CYCLE);
}

/**
 * Convert duty cycle to angle
 */
float duty_cycle_to_angle(float duty_cycle) {
    // Clamp duty cycle to valid range
    if (duty_cycle < SERVO_MIN_DUTY_CYCLE) duty_cycle = SERVO_MIN_DUTY_CYCLE;
    if (duty_cycle > SERVO_MAX_DUTY_CYCLE) duty_cycle = SERVO_MAX_DUTY_CYCLE;
    
    // Linear interpolation to get angle
    return (duty_cycle - SERVO_MIN_DUTY_CYCLE) * 180.0 / (SERVO_MAX_DUTY_CYCLE - SERVO_MIN_DUTY_CYCLE);
}

/**
 * Set PWM duty cycle for a specific servo channel
 * @param channel: Servo channel (0-15)
 * @param duty_cycle: Duty cycle percentage (0.0 to 100.0)
 * Returns: 0 on success, -1 on error
 */
int pca9685_driver_set_servo(int channel, float duty_cycle) {
    // Validate inputs
    if (channel < 0 || channel > 15) {
        printf("Error: Channel must be 0-15, got %d\n", channel);
        return -1;
    }
    
    if (duty_cycle < 0.0 || duty_cycle > 100.0) {
        printf("Error: Duty cycle must be 0.0-100.0%%, got %.1f%%\n", duty_cycle);
        return -1;
    }
    
    // Initialize if not already done
    if (!pca9685_driver_initialized) {
        if (pca9685_driver_init() != 0) {
            return -1;
        }
    }
    
    // Calculate PWM value (0-4095)
    int pwm_value = (int)((duty_cycle / 100.0) * 4096.0);
    if (pwm_value >= 4096) pwm_value = 4095;
    
    // Calculate register addresses for this channel
    int on_l_reg = LED0_ON_L + (channel * 4);
    int on_h_reg = on_l_reg + 1;
    int off_l_reg = on_l_reg + 2;
    int off_h_reg = on_l_reg + 3;
    
    // Write ON_L register = 0x00 (start at 0)
    unsigned char on_l_cmd[2] = {on_l_reg, 0x00};
    if (write(pca9685_driver_fd, on_l_cmd, 2) != 2) {
        perror("Failed to set PWM ON_L");
        return -1;
    }
    
    // Write ON_H register = 0x00
    unsigned char on_h_cmd[2] = {on_h_reg, 0x00};
    if (write(pca9685_driver_fd, on_h_cmd, 2) != 2) {
        perror("Failed to set PWM ON_H");
        return -1;
    }
    
    // Write OFF_L register (lower 8 bits of pwm_value)
    unsigned char off_l_cmd[2] = {off_l_reg, pwm_value & 0xFF};
    if (write(pca9685_driver_fd, off_l_cmd, 2) != 2) {
        perror("Failed to set PWM OFF_L");
        return -1;
    }
    
    // Write OFF_H register (upper 4 bits of pwm_value)
    unsigned char off_h_cmd[2] = {off_h_reg, (pwm_value >> 8) & 0x0F};
    if (write(pca9685_driver_fd, off_h_cmd, 2) != 2) {
        perror("Failed to set PWM OFF_H");
        return -1;
    }
    
    float angle = duty_cycle_to_angle(duty_cycle);
    printf("Channel %d: %.1f%% duty cycle (%.1f degrees)\n", channel, duty_cycle, angle);
    return 0;
}

/**
 * Set servo angle
 * @param channel: Servo channel (0-15)
 * @param angle: Angle in degrees (0-180)
 * Returns: 0 on success, -1 on error
 */
int pca9685_driver_set_servo_angle(int channel, float angle) {
    float duty_cycle = angle_to_duty_cycle(angle);
    return pca9685_driver_set_servo(channel, duty_cycle);
}

/**
 * Clean up and close PCA9685 connection
 */
void pca9685_driver_cleanup() {
    if (pca9685_driver_fd >= 0) {
        close(pca9685_driver_fd);
        pca9685_driver_fd = -1;
        pca9685_driver_initialized = 0;
        printf("PCA9685 connection closed\n");
    }
}

/**
 * Send status message via MQTT
 */
void send_status(int channel, float duty_cycle, const char* status) {
    if (!mosq) return;
    
    json_object *json_obj = json_object_new_object();
    json_object *json_channel = json_object_new_int(channel);
    json_object *json_duty = json_object_new_double(duty_cycle);
    json_object *json_angle = json_object_new_double(duty_cycle_to_angle(duty_cycle));
    json_object *json_status = json_object_new_string(status);
    json_object *json_timestamp = json_object_new_int64(time(NULL));
    
    json_object_object_add(json_obj, "channel", json_channel);
    json_object_object_add(json_obj, "duty_cycle", json_duty);
    json_object_object_add(json_obj, "angle", json_angle);
    json_object_object_add(json_obj, "status", json_status);
    json_object_object_add(json_obj, "timestamp", json_timestamp);
    
    const char *json_string = json_object_to_json_string(json_obj);
    mosquitto_publish(mosq, NULL, MQTT_TOPIC_STATUS, strlen(json_string), json_string, 0, false);
    
    json_object_put(json_obj);
}

/**
 * MQTT message callback
 */
void on_message(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message) {
    printf("Received MQTT message on topic '%s': %s\n", message->topic, (char*)message->payload);
    
    // Parse JSON message
    json_object *json_obj = json_tokener_parse((char*)message->payload);
    if (!json_obj) {
        printf("Error: Invalid JSON message\n");
        return;
    }
    
    // Extract channel and duty cycle
    json_object *json_channel, *json_duty, *json_angle;
    int channel = -1;
    float duty_cycle = -1.0;
    float angle = -1.0;
    
    if (json_object_object_get_ex(json_obj, "channel", &json_channel)) {
        channel = json_object_get_int(json_channel);
    }
    
    if (json_object_object_get_ex(json_obj, "duty_cycle", &json_duty)) {
        duty_cycle = json_object_get_double(json_duty);
    } else if (json_object_object_get_ex(json_obj, "angle", &json_angle)) {
        angle = json_object_get_double(json_angle);
        duty_cycle = angle_to_duty_cycle(angle);
    }
    
    // Validate and apply servo command
    if (channel >= 0 && channel <= 15 && duty_cycle >= 0.0) {
        if (pca9685_driver_set_servo(channel, duty_cycle) == 0) {
            send_status(channel, duty_cycle, "success");
        } else {
            send_status(channel, duty_cycle, "error");
        }
    } else {
        printf("Error: Invalid channel (%d) or duty_cycle (%.2f)\n", channel, duty_cycle);
        send_status(channel, duty_cycle, "invalid_parameters");
    }
    
    json_object_put(json_obj);
}

/**
 * MQTT connection callback
 */
void on_connect(struct mosquitto *mosq, void *userdata, int result) {
    if (result == 0) {
        printf("Connected to MQTT broker\n");
        mosquitto_subscribe(mosq, NULL, MQTT_TOPIC_SERVO, 0);
        printf("Subscribed to topic: %s\n", MQTT_TOPIC_SERVO);
        
        // Send initial status
        send_status(-1, 0.0, "connected");
    } else {
        printf("Failed to connect to MQTT broker: %s\n", mosquitto_strerror(result));
    }
}

/**
 * MQTT disconnect callback
 */
void on_disconnect(struct mosquitto *mosq, void *userdata, int result) {
    printf("Disconnected from MQTT broker\n");
}

/**
 * Initialize MQTT client
 */
int mqtt_init() {
    // Initialize mosquitto library
    mosquitto_lib_init();
    
    // Create mosquitto client instance
    mosq = mosquitto_new(CLIENT_ID, true, NULL);
    if (!mosq) {
        printf("Error: Failed to create mosquitto client\n");
        mosquitto_lib_cleanup();
        return -1;
    }
    
    // Set callbacks
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_disconnect_callback_set(mosq, on_disconnect);
    mosquitto_message_callback_set(mosq, on_message);
    
    // Connect to broker
    int result = mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE);
    if (result != MOSQ_ERR_SUCCESS) {
        printf("Error: Failed to connect to MQTT broker: %s\n", mosquitto_strerror(result));
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return -1;
    }
    
    printf("MQTT client initialized\n");
    return 0;
}

/**
 * Cleanup MQTT client
 */
void mqtt_cleanup() {
    if (mosq) {
        send_status(-1, 0.0, "disconnecting");
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        printf("MQTT client cleaned up\n");
    }
}

/**
 * Main function
 */
int main() {
    printf("PCA9685 MQTT Servo Controller starting...\n");
    
    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Initialize PCA9685
    if (pca9685_driver_init() != 0) {
        printf("Failed to initialize PCA9685\n");
        return 1;
    }
    
    // Initialize MQTT
    if (mqtt_init() != 0) {
        printf("Failed to initialize MQTT\n");
        pca9685_driver_cleanup();
        return 1;
    }
    
    printf("System ready! Listening for MQTT servo commands...\n");
    printf("Send commands to topic: %s\n", MQTT_TOPIC_SERVO);
    printf("Status messages on topic: %s\n", MQTT_TOPIC_STATUS);
    printf("Press Ctrl+C to exit\n\n");
    
    // Main loop
    while (running) {
        int result = mosquitto_loop(mosq, 100, 1);
        if (result != MOSQ_ERR_SUCCESS) {
            printf("MQTT loop error: %s\n", mosquitto_strerror(result));
            if (result == MOSQ_ERR_CONN_LOST) {
                printf("Attempting to reconnect...\n");
                mosquitto_reconnect(mosq);
            }
        }
        usleep(10000); // 10ms delay
    }
    
    printf("\nShutting down...\n");
    mqtt_cleanup();
    pca9685_driver_cleanup();
    
    return 0;
}