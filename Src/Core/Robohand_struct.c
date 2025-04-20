
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "Robohand_struct.h"
#include "pico/mutex.h"

/**
 * @brief Calibration data structure for the BME280 sensor
 */
bme280_calib_data bme280_calib;

/**
 * @brief Calibration data structure for the BME280 sensor
 */
rgb_state rgb_conf;

/**
 * @brief Global structure holding all sensor readings
 */
sensor_data sensor_readings;

/**
 * @brief Global structure containing servo states
 */
servo_motion_profile servo_profiles[NUM_SERVOS];

/**
 * @brief Global structure holding all system status members
 */
system_status sys_status;

/**
 * @brief Initialize the RGB state structure
 * @param[in,out] rgb_struct Pointer to the rgb_state structure to initialize
 */
void init_rgb_state_struct(rgb_state* rgb_struct) {
    if (rgb_struct == NULL) {
        return;
    }
    
    // Initialize all values to safe defaults
    rgb_struct->blink_active = false;
    rgb_struct->blink_state = false;
    rgb_struct->current_r = 0;
    rgb_struct->current_g = 0;
    rgb_struct->current_b = 0;
    rgb_struct->pwm_wrap = 65535;  // Max value for 16-bit PWM
    rgb_struct->blink_interval = 500;  // 500ms default blink interval
    rgb_struct->current_brightness = 0.0f;
    
    // Initialize mutexes
    mutex_init(&rgb_struct->rgb_mutex);
    mutex_init(&rgb_struct->pwm_mutex);
}

/**
 * @brief Initialize the sensor data structure
 * @param[in,out] sensor_struct Pointer to the sensor_data structure to initialize
 */
void init_sensor_data_struct(sensor_data* sensor_struct) {
    if (sensor_struct == NULL) {
        return;
    }
    
    // Initialize all arrays to zero
    memset(sensor_struct->accel, 0, sizeof(sensor_struct->accel));
    memset(sensor_struct->gyro, 0, sizeof(sensor_struct->gyro));
    memset(sensor_struct->mag, 0, sizeof(sensor_struct->mag));
    memset(sensor_struct->adc_values, 0, sizeof(sensor_struct->adc_values));
    
    // Initialize scalar values
    sensor_struct->pressure = 0.0f;
    sensor_struct->altitude = 0.0f;
    sensor_struct->temperature = 0.0f;
    sensor_struct->humidity = 0.0f;
    
    // Initialize mutex
    mutex_init(&sensor_struct->data_mutex);
}

/**
 * @brief Initialize the sensor physical data structure
 * @param[in,out] sensor_struct Pointer to the sensor_data_physical structure to initialize
 */
void init_sensor_data_physical_struct(sensor_data_physical* sensor_struct) {
    if (sensor_struct == NULL) {
        return;
    }
    
    // Initialize all arrays to zero
    memset(sensor_struct->accel, 0, sizeof(sensor_struct->accel));
    memset(sensor_struct->gyro, 0, sizeof(sensor_struct->gyro));
    memset(sensor_struct->mag, 0, sizeof(sensor_struct->mag));
    memset(sensor_struct->adc_values, 0, sizeof(sensor_struct->adc_values));
    
    // Initialize scalar values
    sensor_struct->altitude = 0.0f;
    
    // Initialize mutex
    mutex_init(&sensor_struct->data_mutex);
}

/**
 * @brief Initialize a servo motion profile structure
 * @param[in,out] servo_profile Pointer to the servo_motion_profile structure to initialize
 * @param[in] pwm_pin GPIO pin number to use for this servo
 */
void init_servo_motion_profile_struct(servo_motion_profile* servo_profile, uint8_t pwm_pin) {
    if (servo_profile == NULL) {
        return;
    }
    
    // Initialize with safe default values
    servo_profile->pin = pwm_pin;
    servo_profile->is_moving = false;
    servo_profile->current_pw = 1500;  // Typical center position (1500µs)
    servo_profile->target_pw = 1500;   // Same as current position
    servo_profile->duration_ms = 0;    // No movement duration
    servo_profile->start_time = 0;     // No start time
    
    // Initialize mutex
    mutex_init(&servo_profile->profile_mutex);
}

/**
 * @brief Initialize the system status structure
 * @param[in,out] sys_stat Pointer to the system_status structure to initialize
 */
void init_system_status_struct(system_status* sys_stat) {
    if (sys_stat == NULL) {
        return;
    }
    
    // Initialize with safe default values
    sys_stat->core0_loops = 0;
    sys_stat->core1_loops = 0;
    sys_stat->last_update = 0;
    sys_stat->core0_load = 0.0f;
    sys_stat->core1_load = 0.0f;
    sys_stat->last_watchdog = 0;
    sys_stat->system_ok = true;
    sys_stat->emergency_stop = false;
    
    // Initialize mutex
    mutex_init(&sys_stat->status_mutex);
}

/**
 * @brief Clean up resources used by an RGB state structure
 * @param[in,out] rgb_struct Pointer to the rgb_state structure to destroy
 */
void destroy_rgb_state_struct(rgb_state* rgb_struct) {
    if (rgb_struct == NULL) {
        return;
    }
    
    // Release any resources used by the mutexes
    // Note: In the Raspberry Pi Pico SDK, mutex_deinit() isn't a function
    // but we'll include the logic for potential future compatibility
    
    // Reset all values to safe defaults
    rgb_struct->blink_active = false;
    rgb_struct->current_r = 255;
    rgb_struct->current_g = 0;
    rgb_struct->current_b = 0;
    rgb_struct->current_brightness = 1.0f;
}

/**
 * @brief Clean up resources used by a sensor data structure
 * @param[in,out] sensor_struct Pointer to the sensor_data structure to destroy
 */
void destroy_sensor_data_struct(sensor_data* sensor_struct) {
    if (sensor_struct == NULL) {
        return;
    }
    
    // Reset all values to zero for safety
    memset(sensor_struct->accel, 0, sizeof(sensor_struct->accel));
    memset(sensor_struct->gyro, 0, sizeof(sensor_struct->gyro));
    memset(sensor_struct->mag, 0, sizeof(sensor_struct->mag));
    memset(sensor_struct->adc_values, 0, sizeof(sensor_struct->adc_values));
    
    sensor_struct->pressure = 0.0f;
    sensor_struct->altitude = 0.0f;
    sensor_struct->temperature = 0.0f;
    sensor_struct->humidity = 0.0f;
}

/**
 * @brief Clean up resources used by a physical sensor data structure
 * @param[in,out] sensor_struct Pointer to the sensor_data_physical structure to destroy
 */
void destroy_sensor_data_physical_struct(sensor_data_physical* sensor_struct) {
    if (sensor_struct == NULL) {
        return;
    }
    
    // Reset all values to zero for safety
    memset(sensor_struct->accel, 0, sizeof(sensor_struct->accel));
    memset(sensor_struct->gyro, 0, sizeof(sensor_struct->gyro));
    memset(sensor_struct->mag, 0, sizeof(sensor_struct->mag));
    memset(sensor_struct->adc_values, 0, sizeof(sensor_struct->adc_values));
    
    sensor_struct->altitude = 0.0f;
}

/**
 * @brief Clean up resources used by a servo motion profile structure
 * @param[in,out] servo_profile Pointer to the servo_motion_profile structure to destroy
 */
void destroy_servo_motion_profile_struct(servo_motion_profile* servo_profile) {
    if (servo_profile == NULL) {
        return;
    }
    
    // Stop any active movement for safety
    servo_profile->is_moving = false;
    
    // Reset values to safe defaults
    servo_profile->current_pw = 1500;  // Center position
    servo_profile->target_pw = 1500;   // Same as current position
    servo_profile->duration_ms = 0;    // No movement duration
    servo_profile->start_time = 0;     // No start time
}

/**
 * @brief Clean up resources used by a system status structure
 * @param[in,out] sys_stat Pointer to the system_status structure to destroy
 */
void destroy_system_status_struct(system_status* sys_stat) {
    if (sys_stat == NULL) {
        return;
    }
    
    // Reset counters and flags
    sys_stat->core0_loops = 0;
    sys_stat->core1_loops = 0;
    sys_stat->last_update = 0;
    sys_stat->core0_load = 0.0f;
    sys_stat->core1_load = 0.0f;
    sys_stat->last_watchdog = 0;
    
    // Set system to a safe state before destruction
    sys_stat->emergency_stop = true;
    sys_stat->system_ok = false;
}

 /*!
  * @brief Retrieves the current status of the system.
  * @param[out] dest Pointer to system_status structure to populate.
  */
 void get_system_status(system_status* dest) {
    if (dest == NULL) {
        if (DEBUG > 0) {
            printf("Error: Null pointer in get_system_status\r\n");
        }
        return;
    }

    // Try to get mutex with a timeout
    bool mutex_acquired = mutex_enter_timeout_ms(&sys_status.status_mutex, 50);
    
    if (mutex_acquired) {
        // Get current time for calculations
        uint32_t current_time = time_us_32();
        uint32_t time_elapsed = current_time - sys_status.last_update;
        
        // Copy all data
        memcpy(dest, &sys_status, sizeof(system_status));
        
        // Calculate loads if enough time has passed
        if (time_elapsed > 0) {
            float seconds_elapsed = (float)time_elapsed / 1000000.0f;
            if (seconds_elapsed > 0.001f) { // Avoid division by tiny numbers
                dest->core0_load = (float)sys_status.core0_loops / (seconds_elapsed * 1000.0f);
                dest->core1_load = (float)sys_status.core1_loops / (seconds_elapsed * 1000.0f);
            }
        }
        
        // Reset counters and update timestamp
        sys_status.core0_loops = 0;
        sys_status.core1_loops = 0;
        sys_status.last_update = current_time;
        
        mutex_exit(&sys_status.status_mutex);
    }
    
    else {
        // If we couldn't get the mutex, still provide some data
        memset(dest, 0, sizeof(system_status));
        dest->system_ok = false; // Indicate system issue
        
        if (DEBUG > 0) {
            printf("Warning: Could not acquire system status mutex\r\n");
        }
    }
}



/** @defgroup sensor_data_impl Sensor Data Implementation
*  @brief Implementation of sensor data functions.
*  @{
*/
 
/*!
 * @brief Retrieves current sensor data in a thread-safe manner.
 * @param[out] dest Pointer to sensor_data structure to receive readings.
 * @return true if data copied successfully, false if mutex was busy.
 * @pre dest points to a valid sensor_data object.
 * @warning Caller must allocate destination buffer. Data valid until next update.
 */
bool get_sensor_data(sensor_data* dest) {
    // If user hasn't allocated the array, do it
    if (dest == NULL) {
        init_sensor_data_struct(dest);
        return false;
    }

    mutex_enter_blocking(&sensor_readings.data_mutex);
    memcpy(dest, &sensor_readings, sizeof(sensor_data));
    mutex_exit(&sensor_readings.data_mutex);

    return true;
}

/*!
 * @brief Converts passed sensor data to physical parameters.
 * @param[in] raw Pointer to sensor_data structure to containing unconverted values.
 * @param[out] converted Pointer to sensor_data_physical structure to receive readings as physical parameters.
 * @return true if data copied successfully, false if mutex was busy.
 * @warning Caller must allocate destination buffer. Data valid until next update.
 */
bool convert_sensor_data(const sensor_data* raw, sensor_data_physical* converted) {
    if (!raw || !converted) {
        if (DEBUG > 0) {
            printf("Error: Received NULL parameter\r\n");
        }

        return false;
    }

    // MPU6050 Accelerometer conversion (±2g range: 16384 LSB/g)
    for (int i = 0; i < 3; i++) {
        converted->accel[i] = raw->accel[i] / 16384.0f;
    }

    // MPU6050 Gyroscope conversion (±250dps range: 131 LSB/dps)
    for (int i = 0; i < 3; i++) {
        converted->gyro[i] = raw->gyro[i] / 131.0f;
    }

    // QMC5883L Magnetometer conversion (0.92 mGauss/LSB to µT)
    for (int i = 0; i < 3; i++) {
        converted->mag[i] = raw->mag[i] * 0.92f * 0.1f; // 1 mGauss = 0.1 µT
    }

    // ADC values (already in volts)
    memcpy(converted->adc_values, raw->adc_values, sizeof(raw->adc_values));
    
    // Copy altitude data
    converted->altitude = raw->altitude;

    return true;
}



/** @} */ // end of sensor_data_impl