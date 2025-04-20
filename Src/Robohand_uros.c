/*!
* @file Robohand_uros.c
* @brief MicroROS interface for robotic hand control system.
* @details Facilitates communication with ROS2 systems using MicroROS.
* @author Robert Fudge <rnfudge@mun.ca>
* @date 2025
* @copyright Apache 2.0 License
*/

#include "Robohand_init.h"
#include "Robohand_rgb.h"
#include "Robohand_servos.h"
#include "Robohand_uros.h"

#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>
#include <pico/time.h>

// MicroROS includes
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// MicroROS message types
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/header.h>

// Transport-specific includes
#include <rmw_microros/rmw_microros.h>
#include "./Dependencies/micro_ros_pico_sdk/pico_uart_transports.h"

// MicroROS state
rcl_publisher_t sensor_publisher;
rcl_publisher_t status_publisher;
rcl_subscription_t servo_subscription;

// Message buffers
std_msgs__msg__Float32MultiArray sensor_msg;
std_msgs__msg__String status_msg;
std_msgs__msg__Int32 servo_msg;

// ROS node and support objects
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Timer for publishing sensor data
rcl_timer_t sensor_timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) { printf("MICROROS ERROR: %d\r\n", (int)temp_rc); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) { } }

// Function prototypes
void error_loop();
void servo_callback(const void * msgin);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

int main() {
    // Initialize standard Pico functionality
    stdio_init_all();
    sleep_ms(2000); // Give the system time to stabilize
    
    printf("\nRoboHand MicroROS Interface\r\n");
    
    // Initialize the robotic hand system
    init_robohand_system();
    
    if (HAS_RGB) {
        init_rgb();
        rgb_set_color(0, 0, 255); // Blue indicates initializing
    }
    
    // Initialize MicroROS allocator
    allocator = rcl_get_default_allocator();
    
    // Initialize transport - platform specific
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );
    
    printf("Waiting for agent...\r\n");
    
    // Wait for agent - blink LED while waiting
    if (HAS_RGB) {
        rgb_blink(true, 500);
    }
    
    while (RMW_RET_OK != rmw_uros_ping_agent(1000, 120)) {
        // Monitor loop
        tight_loop_contents();
    }
    
    if (HAS_RGB) {
        rgb_blink(false, 0);
        rgb_set_color(0, 255, 0); // Green indicates connected
    }
    
    printf("Agent found, initializing...\r\n");
    
    // Initialize ROS context
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // Create node
    RCCHECK(rclc_node_init_default(&node, "robohand_node", "", &support));
    
    // Create publishers
    RCCHECK(rclc_publisher_init_default(
        &sensor_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "robohand/sensors"
    ));
    
    RCCHECK(rclc_publisher_init_default(
        &status_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "robohand/status"
    ));
    
    // Create subscribers
    RCCHECK(rclc_subscription_init_default(
        &servo_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "robohand/servo"
    ));
    
    // Create timer
    RCCHECK(rclc_timer_init_default(
        &sensor_timer,
        &support,
        RCL_MS_TO_NS(100),  // 10 Hz
        timer_callback
    ));
    
    // Initialize executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &servo_subscription, 
        &servo_msg, 
        &servo_callback, 
        ON_NEW_DATA
    ));
    
    // Initialize message
    sensor_msg.data.capacity = 20;  // Allocate space for all sensors
    sensor_msg.data.size = 0;
    sensor_msg.data.data = (float*) malloc(sensor_msg.data.capacity * sizeof(float));
    
    status_msg.data.capacity = 256;
    status_msg.data.size = 0;
    status_msg.data.data = (char*) malloc(status_msg.data.capacity * sizeof(char));
    
    printf("MicroROS initialized, entering spin loop\r\n");
    
    // Main loop
    while (1) {
        // Spin executor
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
        
        // Update system counters
        mutex_enter_blocking(&sys_status.status_mutex);
        sys_status.core0_loops++;
        mutex_exit(&sys_status.status_mutex);
        
        sleep_ms(1); // Small sleep to prevent tight loop
    }
    
    // Clean up
    RCCHECK(rcl_publisher_fini(&sensor_publisher, &node));
    RCCHECK(rcl_publisher_fini(&status_publisher, &node));
    RCCHECK(rcl_subscription_fini(&servo_subscription, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rcl_timer_fini(&sensor_timer));
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rclc_support_fini(&support));
    
    free(sensor_msg.data.data);
    free(status_msg.data.data);
    
    return 0;
}

// Error indicator
void error_loop() {
    if (HAS_RGB) {
        rgb_set_color(255, 0, 0); // Red indicates error
        rgb_blink(true, 200);     // Fast blink for error state
    }
    
    while (1) {
        printf("Error: MicroROS failure\r\n");
        sleep_ms(1000);
    }
}

// ROS timer callback for publishing sensor data
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer == NULL) {
        return;
    }
    
    // Read sensor data
    sensor_data local_data;
    sensor_data_physical physical_data;
    
    if (get_sensor_data(&local_data)) {
        convert_sensor_data(&local_data, &physical_data);
        
        //Reset message
        sensor_msg.data.size = 0;
        
        // Add accelerometer data
        for (int i = 0; i < 3; i++) {
            sensor_msg.data.data[sensor_msg.data.size++] = physical_data.accel[i];
        }
        
        // Add gyro data
        for (int i = 0; i < 3; i++) {
            sensor_msg.data.data[sensor_msg.data.size++] = physical_data.gyro[i];
        }
        
        // Add mag data
        for (int i = 0; i < 3; i++) {
            sensor_msg.data.data[sensor_msg.data.size++] = physical_data.mag[i];
        }
        
        // Add ADC values
        for (int i = 0; i < 5; i++) {
            sensor_msg.data.data[sensor_msg.data.size++] = physical_data.adc_values[i];
        }
        
        // Add altitude
        sensor_msg.data.data[sensor_msg.data.size++] = physical_data.altitude;
        
        // Publish
        RCSOFTCHECK(rcl_publish(&sensor_publisher, &sensor_msg, NULL));
    }
    
    // Read system status
    system_status local_status;
    get_system_status(&local_status);
    
    // Format status string
    char status_buffer[256];
    snprintf(status_buffer, 256, 
             "C0:%lu,C1:%lu,OK:%d,ES:%d,L0:%.2f,L1:%.2f",
             local_status.core0_loops, 
             local_status.core1_loops,
             local_status.system_ok ? 1 : 0,
             local_status.emergency_stop ? 1 : 0,
             local_status.core0_load,
             local_status.core1_load);
    
    // Update status message
    status_msg.data.size = strlen(status_buffer);
    memcpy(status_msg.data.data, status_buffer, status_msg.data.size);
    
    // Publish
    RCSOFTCHECK(rcl_publish(&status_publisher, &status_msg, NULL));
}

// ROS subscription callback for servo commands
void servo_callback(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    
    if (msg == NULL) return;
    
    // Command format: SSPPPDDD (Servo, Position, Duration)
    // S: 1 digit for servo index (0-4)
    // P: 4 digits for pulse width (0500-2500)
    // D: 4 digits for duration in ms (0100-9999)
    
    int32_t cmd = msg->data;
    
    if (cmd < 0) {
        // Negative values are interpreted as emergency stop
        mutex_enter_blocking(&sys_status.status_mutex);
        sys_status.emergency_stop = true;
        mutex_exit(&sys_status.status_mutex);
        return;
    }
    
    // Extract command components
    uint8_t servo = (cmd / 100000000) % 10;
    uint16_t position = (cmd / 10000) % 10000;
    uint16_t duration = cmd % 10000;
    
    // Check bounds
    if (servo >= NUM_SERVOS) return;
    if (position < SERVO_MIN_PULSE || position > SERVO_MAX_PULSE) return;
    if (duration < 100 || duration > MAX_MOVE_DURATION_MS) return;
    
    // Execute the command
    actuate_servo(servo, position, duration);
}