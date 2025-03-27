/*!
 * \file Robohand_uros.c
 * \brief micro-ROS wrapper for Robohand control system
 * \author Robert Fudge
 * \date 2025
 */

#include "Robohand.h"
#include "Robohand_uros.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/color_rgba.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
 
//Micro-ROS Components
static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;

//Publishers and subscribers
static rcl_publisher_t sensor_pub;
static rcl_subscription_t servo_sub;
static rcl_timer_t sensor_timer;
static rcl_publisher_t imu_pub;
static rcl_publisher_t mag_pub;
static rcl_publisher_t adc_pub;
static rcl_subscription_t rgb_sub;

//Message instances
static std_msgs__msg__Int32MultiArray servo_cmd_msg;
static std_msgs__msg__Float32MultiArray sensor_msg;
static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__MagneticField mag_msg;
static std_msgs__msg__Float32MultiArray adc_msg;

//Global debug flag
static uint8_t debug = DEBUG;
 
//Transport implementation
#include <uxr/client/profile/transport/custom/custom_transport.h>

//! Time retrieval function for compatibility between uros and pi pico
int clock_gettime(clockid_t clk_id, struct timespec *tp) {
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

//! 
void usleep(uint64_t us) {
    sleep_us(us);
}

//!
bool pico_serial_transport_close(struct uxrCustomTransport* transport) {
    return true;
}

//!
bool pico_serial_transport_open(struct uxrCustomTransport* transport) {
    static bool stdio_initialized = false;
    if(!stdio_initialized) {
        stdio_init_all();
        stdio_initialized = true;
    }
    return true;
}

//!
size_t pico_serial_transport_read(struct uxrCustomTransport* transport, 
                            uint8_t* buf, size_t len, int timeout, uint8_t* errcode) {
    uint64_t start = time_us_64();
    for(size_t i = 0; i < len; ++i) {
        while((time_us_64() - start) < timeout * 1000) {
            int c = getchar_timeout_us(0);
            if(c != PICO_ERROR_TIMEOUT) {
                buf[i] = (uint8_t)c;
                break;
            }
        }
        if((time_us_64() - start) >= timeout * 1000) {
            *errcode = 1;
            return i;
        }
    }
    return len;
}

//!
size_t pico_serial_transport_write(struct uxrCustomTransport* transport, 
                            const uint8_t* buf, size_t len, uint8_t* errcode) {
    for(size_t i = 0; i < len; ++i) {
        if(putchar(buf[i]) != buf[i]) {
            *errcode = 1;
            return i;
        }
    }
    return len;
}
 
/*!
 * \brief Servo command callback
 * \param msg_in Received servo command message
 */
void servo_callback(const void* msg_in) {
    const std_msgs__msg__Int32MultiArray* msg = (const std_msgs__msg__Int32MultiArray*)msg_in;

    if(msg->data.size % 3 != 0) {
        if(debug) printf("Invalid command format. Expected [servo, target, duration]*\n");
        return;
    }

    for(size_t i = 0; i < msg->data.size; i += 3) {
        int servo = msg->data.data[i];
        int target = msg->data.data[i+1];
        int duration = msg->data.data[i+2];
         
        if(servo >= 0 && servo < NUM_SERVOS) {
            if(debug) printf("Moving servo %d to %dµs over %dms\n", 
                    servo, target, duration);
             
            actuate_servo(servo, constrain(target, SERVO_MIN_PULSE, SERVO_MAX_PULSE),
                constrain(duration, 0, MAX_MOVE_DURATION_MS));
        }
    }
}

/*!
 * \brief Updated sensor publishing callback
 */
void sensor_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    sensor_data raw_data;
    sensor_data_physical converted;
    
    if(get_sensor_data(&raw_data) && convert_sensor_data(&raw_data, &converted)) {
        //IMU Data (accel + gyro)
        
        int64_t current_time = rmw_uros_epoch_millis();
        imu_msg.header.stamp.sec = current_time / 1000;
        imu_msg.header.stamp.nanosec = (current_time % 1000) * 1e6;
        imu_msg.linear_acceleration.x = converted.accel[0] * 9.80665;  // g to m/s²
        imu_msg.linear_acceleration.y = converted.accel[1] * 9.80665;
        imu_msg.linear_acceleration.z = converted.accel[2] * 9.80665;
        
        imu_msg.angular_velocity.x = converted.gyro[0] * (M_PI/180);  // deg/s to rad/s
        imu_msg.angular_velocity.y = converted.gyro[1] * (M_PI/180);
        imu_msg.angular_velocity.z = converted.gyro[2] * (M_PI/180);
        
        //Magnetometer (convert from µT to T)
        mag_msg.header.stamp.sec = imu_msg.header.stamp.sec;
        mag_msg.header.stamp.nanosec = imu_msg.header.stamp.nanosec;
        mag_msg.magnetic_field.x = converted.mag[0] * 1e-6;
        mag_msg.magnetic_field.y = converted.mag[1] * 1e-6;
        mag_msg.magnetic_field.z = converted.mag[2] * 1e-6;
        
        //ADC Values
        memcpy(adc_msg.data.data, converted.adc_values, sizeof(converted.adc_values));
        
        //Publish all
        RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
        RCSOFTCHECK(rcl_publish(&mag_pub, &mag_msg, NULL));
        RCSOFTCHECK(rcl_publish(&adc_pub, &adc_msg, NULL));
    }
}

/*!
 * \brief RGB command callback
 */
void rgb_callback(const void* msg_in) {
    const std_msgs__msg__ColorRGBA* msg = (const std_msgs__msg__ColorRGBA*)msg_in;
    
    uint8_t r = (uint8_t)(msg->r * 255);
    uint8_t g = (uint8_t)(msg->g * 255);
    uint8_t b = (uint8_t)(msg->b * 255);
    float brightness = msg->a;  //Use alpha for brightness
    
    rgb_set_brightness(brightness);
    rgb_set_color(r, g, b);
    
    if(debug) printf("Set RGB to: %d,%d,%d @ %.1f%%\n", 
                    r, g, b, brightness*100);
}


int main() {
    rgb_init();
    //Set color (R, G, B values 0-255)
    rgb_set_color(255, 0, 0); //Red

    //Adjust brightness (0.0-1.0)
    rgb_set_brightness(1.0); //50% brightness

    //Initialize transport over USB
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    //Set color (R, G, B values 0-255)
    rgb_set_color(255, 0, 0); //Red
 

    //Set color (R, G, B values 0-255)
    rgb_set_color(255, 128, 0); //Orange
 
    //micro-ROS Initialization
    allocator = rcl_get_default_allocator();

    rgb_set_color(255, 255, 0); //Yellow
     
    //Wait for agent connection
    if (!rmw_uros_ping_agent(1000, 5)) {
        printf("micro-ROS agent not found!\n");
        return 1;
    }

    rgb_set_color(0, 255, 0); //Green
 
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "robohand_node", "", &support));

    rgb_set_color(0, 0, 255); //Cyan
 
    //Create publisher
    RCCHECK(rclc_publisher_init_default(
        &sensor_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "robohand/sensor_data"
    ));
 
    //Create subscriber to recieve servo commands
     RCCHECK(rclc_subscription_init_default(
        &servo_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "robohand/servo_commands"
    ));
 
    //Create timer for sensor checking callback
    RCCHECK(rclc_timer_init_default(
        &sensor_timer,
        &support,
        RCL_MS_TO_NS(SENSOR_PUB_INTERVAL_MS),
        sensor_timer_callback
    ));

    //Create publishers for the Kinematic data
    RCCHECK(rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "robohand/imu"
    ));

    RCCHECK(rclc_publisher_init_default(
        &mag_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "robohand/mag"
    ));

    RCCHECK(rclc_publisher_init_default(
        &adc_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "robohand/adc"
    ));

    //Create RGB subscriber
    RCCHECK(rclc_subscription_init_default(
        &rgb_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
        "robohand/rgb"
    ));

    rgb_set_color(0, 0, 255); //Blue

    //Initialize messages
    //IMU covariance (3x3 row-major)
    for(int i = 0; i < 9; i++) {
        imu_msg.linear_acceleration_covariance[i] = 0;
        imu_msg.angular_velocity_covariance[i] = 0;
    }

    imu_msg.linear_acceleration_covariance[0] = 0.01;  // X variance
    imu_msg.linear_acceleration_covariance[4] = 0.01;  // Y variance
    imu_msg.linear_acceleration_covariance[8] = 0.01;  // Z variance

    //Magnetometer covariance
    for(int i = 0; i < 9; i++) {
        mag_msg.magnetic_field_covariance[i] = 0;
    }
    mag_msg.magnetic_field_covariance[0] = 0.001;
    
    //ADC
    adc_msg.data.capacity = 5;
    adc_msg.data.size = 5;
    adc_msg.data.data = allocator.allocate(sizeof(float)*5, allocator.state);

    static std_msgs__msg__ColorRGBA rgb_cmd_msg;

    //Update executor
    RCCHECK(rclc_executor_add_subscription(
        &executor, &rgb_sub, &rgb_cmd_msg, &rgb_callback, ON_NEW_DATA
    ));
 
    //Initialize executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &servo_sub, &servo_cmd_msg, &servo_callback, ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
 
    //Initialize sensor message
    sensor_msg.data.capacity = SENSOR_DATA_COUNT;
    sensor_msg.data.size = SENSOR_DATA_COUNT;
    sensor_msg.data.data = (float*)allocator.allocate(
        sizeof(float)*SENSOR_DATA_COUNT, 
        allocator.state
    );
     
    if(!sensor_msg.data.data) {
        printf("Failed to allocate sensor message buffer!\n");
        return 1;
    }
 
    printf("micro-ROS node initialized\n");
    rgb_set_color(128, 255, 255); //Magenta

    //Initialize Robohand system (launches Core 1)
    init_robohand_system();
 
    //Main loop
    while(true) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
        sleep_us(1000);
    }
 
    //Cleanup (unreachable in normal operation)
    RCCHECK(rcl_publisher_fini(&sensor_pub, &node));
    RCCHECK(rcl_subscription_fini(&servo_sub, &node));
    RCCHECK(rcl_node_fini(&node));
 
    return 0;
}

