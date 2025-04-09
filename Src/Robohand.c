/*!
 * @file Robohand.c
 * @brief Implementation of robotic hand control system with dual-core operation.
 * @details Manages hardware interfaces, sensor data collection, and servo control on Core 1.
 * @author Robert Fudge
 * @date 2025
 * @copyright Apache 2.0 License
 */

#include "Robohand.h"
#include "Robohand_advanced.h"

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

//Gamma correction table (gamma = 2.8) for brightness smoothing
const uint8_t gamma_table[256] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,
    2,   2,   2,   3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,
    6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  11,  11,
    12,  12,  13,  13,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  20,
    20,  21,  22,  22,  23,  24,  24,  25,  26,  27,  27,  28,  29,  30,  31,  31,
    32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,
    49,  50,  51,  52,  53,  55,  56,  57,  59,  60,  61,  63,  64,  65,  67,  68,
    70,  71,  73,  74,  76,  78,  79,  81,  83,  84,  86,  88,  90,  91,  93,  95,
    97,  99,  101, 103, 105, 107, 109, 111, 113, 115, 117, 120, 122, 124, 126, 129,
    131, 134, 136, 138, 141, 143, 146, 148, 151, 154, 156, 159, 162, 165, 167, 170,
    173, 176, 179, 182, 185, 188, 191, 194, 197, 200, 204, 207, 210, 213, 217, 220,
    224, 227, 231, 234, 238, 241, 245, 249, 252, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255
};

//Global variables - All files
servo_motion_profile servo_profiles[NUM_SERVOS];                    //! Structure array containing the servo profiles
system_status sys_status;                                           //! Structure containing system status
const uint SERVO_PINS[NUM_SERVOS] = {11, 12, 13, 14, 15};           //! GPIO pins to use for mechanical actuation
const float VOLTAGE_DIVIDER_RATIO = 1.0f;                           //! Ratio for voltage calculations (Not sure if this is needed anymore)

//Global variables - This file only, only initialized once
static const float PWM_US_TO_LEVEL = 39062.0f / 20000.0f;           //! Constant value used for PWM conversion for servos
static absolute_time_t prev_update_time;                            //! Used to track the time since last update

//Dedicated hardware mutexes
static mutex_t servo_mutex;                                         //! Mutex protecting Servo access

static struct repeating_timer hb_timer;                             //! Timer for ADC callback (Default 500ms / 2Hz)
static struct repeating_timer blink_timer;                          //! Timer for triggering the blink callback
static rgb_state rgb_conf;                                          //! Global struct for configuring the Common Cathode RGB


//Forward function declarations
//Struct interactors
static void destroy_rgb_state_struct(rgb_state* rgb_struct);
static void destroy_sensor_data_struct(sensor_data* sensor_struct);
static void destroy_sensor_data_physical_struct(sensor_data_physical* sensor_struct);
static void destroy_servo_motion_profile_struct(servo_motion_profile* servo_profile);
static void destroy_system_status_struct(system_status* sys_status);

static void init_rgb_state_struct(rgb_state* rgb_struct);
static void init_sensor_data_struct(sensor_data* sensor_struct);
static void init_sensor_data_physical_struct(sensor_data_physical* sensor_struct);
static void init_servo_motion_profile_struct(servo_motion_profile* servo_profile, uint8_t pwm_pin);
static void init_system_status_struct(system_status* sys_status);

//Servo setup functions
static void handle_servo_commands(void);
static void init_servo_pwm(void);
static void update_servo_positions(void);

//Helper functions
static bool blink_callback(const struct repeating_timer *t);
static bool heartbeat_callback(const struct repeating_timer* t);

/**
 * 
 * User facing function definitions
 * 
 */

//! Move the servo with the desired settings
void actuate_servo(uint8_t servo, uint16_t pulse_width, uint16_t duration_ms) {
    if (HAS_SERVOS) {
        if(servo >= NUM_SERVOS) return;
    
        //Validate inputs
        pulse_width = constrain_u16(pulse_width, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
        duration_ms = constrain_u16(duration_ms, 0, MAX_MOVE_DURATION_MS);

        //Pack command
        uint32_t command = ((uint32_t)servo << 28) | ((pulse_width & 0xFFF) << 16) | (duration_ms & 0xFFFF);
        multicore_fifo_push_blocking(command);
    }
}

//!
uint16_t constrain_u16(uint16_t value, uint16_t min, uint16_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

//! Convert data to physical parameters
bool convert_sensor_data(const sensor_data* raw, sensor_data_physical* converted) {
    if (!raw || !converted) {
        return false;
    }

    // MPU6050 Accelerometer conversion (±8g range: 4096 LSB/g)
    for (int i = 0; i < 3; i++) {
        converted->accel[i] = raw->accel[i] / 4096.0f;
    }

    // MPU6050 Gyroscope conversion (±500dps range: 65.5 LSB/dps)
    for (int i = 0; i < 3; i++) {
        converted->gyro[i] = raw->gyro[i] / 65.5f;
    }

    //QMC5883L Magnetometer conversion (0.92 mGauss/LSB to µT)
    for (int i = 0; i < 3; i++) {
        converted->mag[i] = raw->mag[i] * 0.92f * 0.1f; // 1 mGauss = 0.1 µT
    }

    //ADC values (already in volts)
    memcpy(converted->adc_values, raw->adc_values, sizeof(raw->adc_values));

    return true;
}

//Core 1 config will need to be different between pico and pico w
#if defined(PICO_BOARD_pico_w) || defined(PICO_BOARD_IS_PICO_W)
    //Pico W will likely need its own core1 config
#else

//! Core 1 main routine
void core1_entry(void) {
    uint32_t loop_count = 0;

    robohand_init_components();

    //Hardware initialization
    if (HAS_SERVOS) {
        if (DEBUG > 0) {
            printf("Initializing Servos.\r\n");
        }

        init_servo_pwm();
        mutex_init(&servo_mutex);

        if (DEBUG > 0) {
            printf("Servo init complete.\r\n");
        }
    }
    
    add_repeating_timer_ms(-1000, &heartbeat_callback, NULL, &hb_timer); //1Hz - 1s
    init_system_status_struct(&sys_status);
    multicore_fifo_push_blocking(1);

    //Initialize watchdog
    watchdog_enable(8* 1000, true); //8s timeout

    if (DEBUG > 0) {
        printf("Starting main loop.\r\n");
    }

    absolute_time_t last_load_update = get_absolute_time();

    while(1) {
        loop_count++;
        
        // Update every second, this is not doing what I want it to
        if (absolute_time_diff_us(last_load_update, get_absolute_time()) > 1000000) {
            mutex_enter_blocking(&sys_status.status_mutex);
            sys_status.core1_loops = loop_count;
            last_load_update = get_absolute_time();
            mutex_exit(&sys_status.status_mutex);
            loop_count = 0;
        }

        robohand_read();
        handle_servo_commands();
        update_servo_positions();

        watchdog_update(); //Reset watchdog timer
        sleep_ms(50);
    }
}

#endif

//! Get a copy of the sensor data structure
bool get_sensor_data(sensor_data* dest) {
    //If user hasn't allocated the array, do it
    if (dest == NULL) {
        init_sensor_data_struct(dest);

        if (dest != NULL && !mutex_enter_timeout_ms(&sensor_readings.data_mutex, 25)) { //25ms
            memcpy(dest, &sensor_readings, sizeof(sensor_data));
            mutex_exit(&sensor_readings.data_mutex);
            return true;
        }
    }
    
    return false;
}

//! Get the status of desired servo
bool get_servo_status(uint8_t servo, servo_motion_profile* dest) {
    if (HAS_SERVOS) {
        if(servo >= NUM_SERVOS) {
            return false;
        }

        //If user hasn't allocated the array, do it
        if (dest == NULL) {
            init_servo_motion_profile_struct(dest, 0);
        }

        if(!mutex_enter_timeout_ms(&servo_profiles[servo].profile_mutex, 25)) { //25ms
            memcpy(dest, &servo_profiles[servo], sizeof(servo_motion_profile));
            mutex_exit(&servo_profiles[servo].profile_mutex);
            return true;
        }
    }

    return false;
}

//! Thread-safe status getter
void get_system_status(system_status* dest) {
    mutex_enter_blocking(&sys_status.status_mutex);
    *dest = sys_status;  //Atomic copy of entire structure
    sys_status.core0_loops = 0; //reset core members
    sys_status.core1_loops = 0;
    sys_status.last_update = time_us_32();
    mutex_exit(&sys_status.status_mutex);
}

//! Common Cathode RGB LED initialization function
void init_rgb(void) {
    if (HAS_RGB) {
        //Initialize PWM
        gpio_set_function(RGB_RED_PIN, GPIO_FUNC_PWM);
        gpio_set_function(RGB_GREEN_PIN, GPIO_FUNC_PWM);
        gpio_set_function(RGB_BLUE_PIN, GPIO_FUNC_PWM);

        pwm_config config = pwm_get_default_config();
        pwm_config_set_wrap(&config, rgb_conf.pwm_wrap);
        pwm_config_set_clkdiv(&config, 4.f);
    
        //Get the slices for each pin
        uint slices[] = {
            pwm_gpio_to_slice_num(RGB_RED_PIN),
            pwm_gpio_to_slice_num(RGB_GREEN_PIN),
            pwm_gpio_to_slice_num(RGB_BLUE_PIN)
        };
    
        //Initialize the PWM pins
        for(int i = 0; i < 3; i++) {
            pwm_init(slices[i], &config, true);
        }

        init_rgb_state_struct(&rgb_conf);
    
        //Turn the LED off
        rgb_set_color(0, 0, 0);
    }
    
}

//! System initialization function
void init_robohand_system(void) {
    //Initialize shared data structures and mutexes for hardware resources
    init_sensor_data_struct(&sensor_readings);
     
    //Launch core1 with sensor handling
    multicore_launch_core1(core1_entry);

    printf("Waiting for core1 init.\r\n");
     
    //Wait for core1 initialization
    while(!multicore_fifo_rvalid());

    printf("Done waiting for core1 init.\r\n");

    multicore_fifo_pop_blocking();
}

//! User-facing blink control function
void rgb_blink(bool enable, uint32_t interval_ms) {
    if (HAS_RGB) {
        mutex_enter_blocking(&rgb_conf.rgb_mutex);

        //(Re)start timer if not active or interval changed
        if(rgb_conf.blink_active) {
            cancel_repeating_timer(&blink_timer);
        }
    
        if (enable) {
            if(!rgb_conf.blink_active || rgb_conf.blink_interval != interval_ms) {
                rgb_conf.blink_active = true;
                rgb_conf.blink_interval = interval_ms;
                add_repeating_timer_ms(-(int32_t)interval_ms, &blink_callback, NULL, &blink_timer);
            }
        }
        
        else {
            if(rgb_conf.blink_active) {
                //Stop blinking and restore color
                rgb_conf.blink_active = false;
                cancel_repeating_timer(&blink_timer);
                rgb_set_color(rgb_conf.current_r, rgb_conf.current_g, rgb_conf.current_b);
            }
        }
    
        mutex_exit(&rgb_conf.rgb_mutex);
    }
}

//! Function for setting brightness of Common Cathode RGB LED
void rgb_set_brightness(float brightness) {
    if (HAS_RGB) {
        mutex_enter_blocking(&rgb_conf.rgb_mutex);

        rgb_conf.current_brightness = fmaxf(0.0f, fminf(1.0f, brightness));

        //Apply brightness and gamma correction
        uint16_t red = gamma_table[(uint8_t)(rgb_conf.current_r * rgb_conf.current_brightness)];
        uint16_t green = gamma_table[(uint8_t)(rgb_conf.current_g * rgb_conf.current_brightness)];
        uint16_t blue = gamma_table[(uint8_t)(rgb_conf.current_b * rgb_conf.current_brightness)];

        mutex_exit(&rgb_conf.rgb_mutex);

        //Set PWM levels
        mutex_enter_blocking(&rgb_conf.pwm_mutex);
        pwm_set_gpio_level(RGB_RED_PIN, red);
        pwm_set_gpio_level(RGB_GREEN_PIN, green);
        pwm_set_gpio_level(RGB_BLUE_PIN, blue);
        mutex_exit(&rgb_conf.pwm_mutex);
    }
}


//! Function for setting red, green, and blue values of Common Cathode RGB LED
void rgb_set_color(uint8_t r, uint8_t g, uint8_t b) {
    if (HAS_RGB) {
        mutex_enter_blocking(&rgb_conf.rgb_mutex);
        //Store original values before brightness adjustment
        rgb_conf.current_r = r;
        rgb_conf.current_g = g;
        rgb_conf.current_b = b;

        //Apply brightness and gamma correction
        uint16_t red = gamma_table[(uint8_t)(rgb_conf.current_r * rgb_conf.current_brightness)];
        uint16_t green = gamma_table[(uint8_t)(rgb_conf.current_g * rgb_conf.current_brightness)];
        uint16_t blue = gamma_table[(uint8_t)(rgb_conf.current_b * rgb_conf.current_brightness)];
        mutex_exit(&rgb_conf.rgb_mutex);

        //Set PWM levels
        mutex_enter_blocking(&rgb_conf.pwm_mutex);
        pwm_set_gpio_level(RGB_RED_PIN, red);
        pwm_set_gpio_level(RGB_GREEN_PIN, green);
        pwm_set_gpio_level(RGB_BLUE_PIN, blue);
        mutex_exit(&rgb_conf.pwm_mutex);
    }
}

/** @defgroup struct_interact Robohand Structure Interactors
 *  @brief These functions are designed to set required data structures to well-known state
 *  @{
 */

 /*!
 * @brief RGB state destructor.
 * @details Destroys structure by freeing required memory.
 * @param[in] rgb_struct Populated, valid data structure.
 */
static void destroy_rgb_state_struct(rgb_state* rgb_struct) {
    if (rgb_struct != NULL) {
        free(rgb_struct);
    }
}

/*!
 * @brief Sensor data destructor.
 * @details Destroys the system sensor data structure.
 * @param[in] sensor_struct Populated, valid data structure.
 */
static void destroy_sensor_data_struct(sensor_data* sensor_struct) {
    if (sensor_struct != NULL) {
        free(sensor_struct);
    }
}

/*!
 * @brief Physical sensor data destructor.
 * @details Destroys the physical sensor data structure.
 * @param[in] sensor_struct Populated, valid data structure.
 */
static void destroy_sensor_data_physical_struct(sensor_data_physical* sensor_struct) {
    if (sensor_struct != NULL) {
        free(sensor_struct);
    }
}

/*!
 * @brief Servo motion profile destructor.
 * @details Destroys the servo motion profile data structure.
 * @param[in] servo_profile Populated, valid data structure.
 */
static void destroy_servo_motion_profile_struct(servo_motion_profile* servo_profile) {
    if (servo_profile != NULL) {
        free(servo_profile);
    }
}

/*!
 * @brief System status data destructor.
 * @details Destroys the system status data structure.
 * @param[in] sys_status Populated, valid data structure.
 */
static void destroy_system_status_struct(system_status* system_status) {
    if (system_status != NULL) {
        free(system_status);
    }
}

/*!
 * @brief Initialize RGB state structure.
 * @details Sets structure members to a well known state. If memory has not been allocated, the system will allocate it.
 * @param[out] rgb_struct Populated, valid data structure.
 */
static void init_rgb_state_struct(rgb_state* rgb_struct) {
    if (rgb_struct == NULL) {
        rgb_struct = (rgb_state*) malloc(1 * sizeof(rgb_state));
    }

    rgb_struct->blink_active = false;
    rgb_struct->blink_state = false;
    rgb_struct->current_r = 0;
    rgb_struct->current_g = 0;
    rgb_struct->current_b = 0;
    rgb_struct->pwm_wrap = 0;
    rgb_struct->blink_interval = 1000;
    rgb_struct->current_brightness = 1.0;

    mutex_init(&rgb_struct->rgb_mutex);
    mutex_init(&rgb_struct->pwm_mutex);
}

/*!
 * @brief Initialize sensor data structure (internal).
 * @details Sets structure members to well-known state.
 * @param[out] sensor_struct Intitialized sensor structure.
 */
static void init_sensor_data_struct(sensor_data* sensor_struct) {
    if (sensor_struct == NULL) {
        sensor_struct = (sensor_data*) malloc(1 * sizeof(sensor_data));
    }

    sensor_struct->accel[0] = ~0;
    sensor_struct->accel[1] = ~0;
    sensor_struct->accel[2] = ~0;

    sensor_struct->gyro[0] = ~0;
    sensor_struct->gyro[1] = ~0;
    sensor_struct->gyro[2] = ~0;

    sensor_struct->mag[0] = ~0;
    sensor_struct->mag[1] = ~0;
    sensor_struct->mag[2] = ~0;

    sensor_struct->adc_values[0] = (float)~0;
    sensor_struct->adc_values[1] = (float)~0;
    sensor_struct->adc_values[2] = (float)~0;
    sensor_struct->adc_values[3] = (float)~0;
    sensor_struct->adc_values[4] = (float)~0;

    mutex_init(&sensor_struct->data_mutex);
}

/*!
 * @brief Initialize physical sensor data structure.
 * @details Sets structure members to well-known state.
 * @param[out] sensor_struct Initialized sensor data structure.
 */
static void init_sensor_data_physical_struct(sensor_data_physical* sensor_struct) {
    if (sensor_struct == NULL) {
        sensor_struct = (sensor_data_physical*) malloc(1 * sizeof(sensor_data_physical));
    }

    sensor_struct->accel[0] = (float)~0;
    sensor_struct->accel[1] = (float)~0;
    sensor_struct->accel[2] = (float)~0;

    sensor_struct->gyro[0] = (float)~0;
    sensor_struct->gyro[1] = (float)~0;
    sensor_struct->gyro[2] = (float)~0;

    sensor_struct->mag[0] = (float)~0;
    sensor_struct->mag[1] = (float)~0;
    sensor_struct->mag[2] = (float)~0;

    sensor_struct->adc_values[0] = (float)~0;
    sensor_struct->adc_values[1] = (float)~0;
    sensor_struct->adc_values[2] = (float)~0;
    sensor_struct->adc_values[3] = (float)~0;
    sensor_struct->adc_values[4] = (float)~0;

    mutex_init(&sensor_struct->data_mutex);
}

/*!
 * @brief Intitialize a servo motion profile.
 * @details Retries write three times before quitting.
 * @param[out] servo_profile Initialized servo profile.
 * @param[in] pwm_pin Pin to use for PWM signal modulation.
 */
static void init_servo_motion_profile_struct(servo_motion_profile* servo_profile, uint8_t pwm_pin) {
    if (servo_profile == NULL) {
        servo_profile = (servo_motion_profile*) malloc(1 * sizeof(servo_motion_profile));
    }

    servo_profile->pin = pwm_pin;
    servo_profile->is_moving = false;
    servo_profile->current_pw = 1500;
    servo_profile->target_pw = 1500;
    servo_profile->duration_ms = 1000;
    servo_profile->start_time = time_us_32();
    
    mutex_init(&servo_profile->profile_mutex);
}

/*!
 * @brief Initialize system status structure.
 * @details Used to hold information for system state/performance metrics.
 * @param[out] sys_stat Initialized system status structure.
 */
static void init_system_status_struct(system_status* sys_stat) {
    if (sys_stat == NULL) {
        sys_stat = (system_status*) malloc(1 * sizeof(system_status));
    }

    sys_stat->core0_loops = 0;
    sys_stat->core1_loops = 0;
    sys_stat->last_update = time_us_32();
    sys_stat->last_watchdog = time_us_32();
    sys_stat->system_ok = true;
    sys_stat->emergency_stop = false;

    mutex_init(&sys_stat->status_mutex);
}

/** @} */ // end of struct_interact

/** @defgroup servo_ctl Servo Control
 *  @brief Used to control servos.
 *  @{
 */


/*!
 * @brief Function that unpacks commands sent from other modules.
 * @details Reads from the FIFO and parses servo commands.
 * @post The appropriate servo profile is updated.
 */
static void handle_servo_commands(void) {
    if (HAS_SERVOS) {
        while(multicore_fifo_rvalid()) {
            uint32_t cmd = multicore_fifo_pop_blocking();
            uint8_t servo = (cmd >> 28) & 0x0F;         //Extract top 4 bits for servo
            uint16_t pulse_width = (cmd >> 16) & 0xFFF; //Next 12 bits for pulse width
            uint16_t duration = cmd & 0xFFFF;           //Lower 16 bits for duration

            if(servo < NUM_SERVOS && mutex_try_enter(&servo_mutex, NULL)) {
                servo_profiles[servo].target_pw = pulse_width;
                servo_profiles[servo].duration_ms = duration;
                servo_profiles[servo].start_time = time_us_32();
                servo_profiles[servo].is_moving = true;
                mutex_exit(&servo_mutex);
            }
        }
    }
}

/*!
 * @brief Initialize PWM for servo control.
 * @details Configures PWM frequency to 50Hz (20ms period) for standard servos.
 * @post All servo pins are configured with correct PWM settings.
 */
static void init_servo_pwm(void) {
    if (HAS_SERVOS) {
        for(int i = 0; i < NUM_SERVOS; i++) {
            gpio_set_function(SERVO_PINS[i], GPIO_FUNC_PWM);
            uint slice_num = pwm_gpio_to_slice_num(SERVO_PINS[i]);
            pwm_config config = pwm_get_default_config();
            pwm_config_set_clkdiv(&config, 64.0f);  //125MHz / 64 = 1.953125MHz
            pwm_config_set_wrap(&config, 39062);    //1.953125MHz / 39062 ≈ 50Hz
            pwm_init(slice_num, &config, true);
        }
    }
}

/*!
 * @brief Update servo positions using motion profiles.
 * @details Calculates smooth transitions between current and target positions.
 * @post Servo PWM outputs are updated with new calculated positions.
 */
static void update_servo_positions(void) {
    absolute_time_t now = get_absolute_time();
    absolute_time_t dt_us = absolute_time_diff_us(prev_update_time, now);
    
    if(dt_us <= 0) {
        return;  //Prevent time travel
    }
    
    for(int i = 0; i < NUM_SERVOS; i++) {
        if(mutex_try_enter(&servo_mutex, NULL)) {
            if(!servo_profiles[i].is_moving) {
                mutex_exit(&servo_mutex);
                continue;
            }

            uint32_t elapsed = time_us_32() - servo_profiles[i].start_time;
            float t = (float)elapsed / (servo_profiles[i].duration_ms * 1000);
            
            if(t >= 1.0f) {
                servo_profiles[i].current_pw = servo_profiles[i].target_pw;
                servo_profiles[i].is_moving = false;
            }
                
            else {
                //Quintic easing for smoother motion
                float eased_t = t * t * t * (t * (6 * t - 15) + 10);
                int32_t delta = servo_profiles[i].target_pw - servo_profiles[i].current_pw;
                servo_profiles[i].current_pw += (uint16_t)((float)delta * eased_t);
                servo_profiles[i].current_pw = constrain_u16(servo_profiles[i].current_pw, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
            }

            //Update PWM (50Hz = 20ms period)
            uint16_t level = (uint16_t)(servo_profiles[i].current_pw * PWM_US_TO_LEVEL);
            pwm_set_gpio_level(SERVO_PINS[i], level);
            
            mutex_exit(&servo_mutex);
        }

        prev_update_time = now;
    }
}

/** @} */ // end of servo_ctl

/** @defgroup misc Miscellaneous
 *  @brief Helper functions providing utility to developers.
 *  @{
 */

/*!
 * @brief Callback allowing for toggling of RGB functionality.
 * @return Always returns true to continue the timer.
 */
static bool blink_callback(const struct repeating_timer *t) {
    (void)t;

    if (HAS_RGB && mutex_try_enter(&rgb_conf.rgb_mutex, NULL) && rgb_conf.blink_active) {
        rgb_conf.blink_state = !rgb_conf.blink_state;
                
        if(rgb_conf.blink_state) {
            //Restore original color
            pwm_set_gpio_level(RGB_RED_PIN, gamma_table[(uint8_t)(rgb_conf.current_r * rgb_conf.current_brightness)]);
            pwm_set_gpio_level(RGB_GREEN_PIN, gamma_table[(uint8_t)(rgb_conf.current_g * rgb_conf.current_brightness)]);
            pwm_set_gpio_level(RGB_BLUE_PIN, gamma_table[(uint8_t)(rgb_conf.current_b * rgb_conf.current_brightness)]);
        }
            
        else {
            //Turn off LEDs
            pwm_set_gpio_level(RGB_RED_PIN, 0);
            pwm_set_gpio_level(RGB_GREEN_PIN, 0);
            pwm_set_gpio_level(RGB_BLUE_PIN, 0);
        }

        mutex_exit(&rgb_conf.rgb_mutex);
    }
    
    return true;
}

/*!
 * @brief System heartbeat callback.
 * @param t Pointer to repeating timer structure.
 * @return Always returns true to continue timer.
 * @details Toggles onboard LED to indicate system liveliness.
 */
static bool heartbeat_callback(const struct repeating_timer* t) {
    (void)t;

    static bool led_state = false;
    
    //Needs to be macro, because regular Pico target doesnt have these functions
    #if defined(PICO_BOARD_IS_PICO_W)
    cyw43_arch_gpio_put(ROBOHAND_LED_PIN, led_state);
    #else
    gpio_put(ROBOHAND_LED_PIN, led_state);
    #endif
    
    led_state = !led_state;
    return true;
}