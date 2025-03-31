/*!
 * @file Robohand.c
 * @brief Implementation of robotic hand control system with dual-core operation.
 * @details Manages hardware interfaces, sensor data collection, and servo control on Core 1.
 * @author Robert Fudge
 * @date 2025
 * @copyright Apache 2.0 License
 */

#include "Robohand.h"

#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

//Macros for I2C flag
#define MPU_READ_FLAG (1 << 0)          ///< Bit signalling an MPU read is needed over the I2C bus
#define HMC_READ_FLAG (1 << 1)          ///< Bit signalling an HMC read is needed over the I2C bus
#define ADC_READ_FLAG (1 << 2)          ///< Bit signalling an ADC read is needed over the I2C bus

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

//Static bools to prevent resource contention of missing components
static bool ads_written = false;                                    //! Will skip trying to re-acquire and write to sensor mutex if the ADS1115 is disconnected
static bool hmc5883l_written = false;                               //! Will skip trying to re-acquire and write to sensor mutex if the HMC5883L is disconnected
static bool mpu6050_written = false;                                //! Will skip trying to re-acquire and write to sensor mutex if the MPU6050 is disconnected
static bool pico_adc_written = false;                               //! Will skip trying to re-acquire and write to sensor mutex if the Pi Pico ADC is unused

//Global variables - This file only, only initialized once
static const float PWM_US_TO_LEVEL = 39062.0f / 20000.0f;           //! Constant value used for PWM conversion for servos
static absolute_time_t prev_update_time;                            //! Used to track the time since last update

//Dedicated hardware mutexes
static mutex_t i2c_mutex;                                           //! Mutex protecting I2C access
static mutex_t servo_mutex;                                         //! Mutex protecting Servo access

static struct repeating_timer adc_timer;                            //! Timer for ADC callback (Default 100ms / 10Hz)
static struct repeating_timer hb_timer;                             //! Timer for ADC callback (Default 500ms / 2Hz)
static struct repeating_timer mpu_timer;                            //! Timer for ADC callback (Default 50ms / 20Hz)
static struct repeating_timer blink_timer;                          //! Timer for triggering the blink callback
static rgb_state rgb_conf;                                          //! Global struct for configuring the Common Cathode RGB
static sensor_data sensor_readings;                                 //! Mutex protected structure containing sensor information

static volatile uint8_t i2c_operation_flags = 0;                    //! Register coordinating I2C accesses, may change at any point

/*! \brief Static function prototypes **/
static bool adc_sample_callback(struct repeating_timer* t);
static float ads_voltage(uint16_t raw);
static bool blink_callback(struct repeating_timer *t);
static void destroy_rgb_state_struct(rgb_state* rgb_struct);
static void destroy_sensor_data_struct(sensor_data* sensor_struct);
static void destroy_sensor_data_physical_struct(sensor_data_physical* sensor_struct);
static void destroy_servo_motion_profile_struct(servo_motion_profile* servo_profile);
static void destroy_system_status_struct(system_status* sys_status);
static void ads1115_drdy_handler(uint gpio, uint32_t events);
static void gy271_drdy_handler(uint gpio, uint32_t events);
static void mpu6050_alert_handler(uint gpio, uint32_t events);
static void handle_servo_commands(void);
static bool heartbeat_callback(struct repeating_timer* t);
static bool i2c_write_with_retry(uint8_t addr, const uint8_t* src, size_t len);
static void init_rgb_state_struct(rgb_state* rgb_struct);
static void init_sensor_data_struct(sensor_data* sensor_struct);
static void init_sensor_data_physical_struct(sensor_data_physical* sensor_struct);
static void init_servo_motion_profile_struct(servo_motion_profile* servo_profile, uint8_t pwm_pin);
static void init_servo_pwm(void);
static void init_system_status_struct(system_status* sys_status);
static void init_watchdog(void);
static bool mpu6050_callback(struct repeating_timer* t);
static float read_adc2(void);
static uint16_t read_ads_channel(uint8_t channel);
static void read_adc_data(void);
static void read_hmc5883l_data(void);
static void read_mpu6050_data(void);
static void update_servo_positions(void);

//! Move the servo with the desired settings
void actuate_servo(uint8_t servo, uint16_t pulse_width, uint16_t duration_ms) {
    if (HAS_SERVOS) {
        if(servo >= NUM_SERVOS) return;
    
        //Validate inputs
        pulse_width = constrain(pulse_width, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
        duration_ms = constrain(duration_ms, 0, MAX_MOVE_DURATION_MS);

        //Pack command
        uint32_t command = ((uint32_t)servo << 28) | ((pulse_width & 0xFFF) << 16) | (duration_ms & 0xFFFF);
        multicore_fifo_push_blocking(command);
    }
}

//! Convert data to physical parameters
bool convert_sensor_data(const sensor_data* raw, sensor_data_physical* converted) {
    if (!raw || !converted) {
        return false;
    }

    //MPU6050 Accelerometer conversion (±2g range: 16384 LSB/g)
    for (int i = 0; i < 3; i++) {
        converted->accel[i] = raw->accel[i] / 16384.0f;
    }

    //MPU6050 Gyroscope conversion (±250dps range: 131 LSB/dps)
    for (int i = 0; i < 3; i++) {
        converted->gyro[i] = raw->gyro[i] / 131.0f;
    }

    //HMC5883L Magnetometer conversion (0.92 mGauss/LSB to µT)
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
    absolute_time_t next_report = get_absolute_time();

    init_system_comms();
    
    add_repeating_timer_ms(-1000, heartbeat_callback, NULL, &hb_timer); //1Hz - 1s

    //Initialize status struct
    init_system_status_struct(&sys_status);

    //Signal readiness
    multicore_fifo_push_blocking(1);

    //Initialize watchdog
    init_watchdog();

    if (DEBUG) {
        printf("Starting main loop.\r\n");
    }


    while(1) {
        loop_count++;
        
        //Update status
        mutex_try_enter(&sys_status.status_mutex, NULL);
        sys_status.core1_loops += loop_count;
        sys_status.last_watchdog = time_us_32();
        sys_status.system_ok = !watchdog_caused_reboot();
        loop_count = 0;
        mutex_exit(&sys_status.status_mutex);

        //Handle sensor reads via flags
        if((i2c_operation_flags & MPU_READ_FLAG) && HAS_I2C) {
            read_mpu6050_data();
            i2c_operation_flags &= ~MPU_READ_FLAG;
        }
        
        if((i2c_operation_flags & HMC_READ_FLAG) && HAS_I2C) {
            read_hmc5883l_data();
            i2c_operation_flags &= ~HMC_READ_FLAG;
        }

        if((i2c_operation_flags & ADC_READ_FLAG) && HAS_I2C) {
            read_adc_data();
            i2c_operation_flags &= ~ADC_READ_FLAG;
        }

        watchdog_update(); //Reset watchdog timer
    }
}

#endif

//! Get a copy of the sensor data structure
bool get_sensor_data(sensor_data* dest) {
    //If user hasn't allocated the array, do it
    if (dest == NULL) {
        init_sensor_data_struct(dest);
    }

    if(!mutex_enter_timeout_ms(&sensor_readings.data_mutex, 25)) { //25ms
        memcpy(dest, &sensor_readings, sizeof(sensor_data));
        mutex_exit(&sensor_readings.data_mutex);
        return true;
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
    sys_status.last_reset_core0 = time_us_32();
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

    printf("Waiting for core1 init");
     
    //Wait for core1 initialization
    while(!multicore_fifo_rvalid());

    printf("Done waiting for core1 init");

    multicore_fifo_pop_blocking();
}

//! User-facing blink control function
void rgb_blink(bool enable, uint32_t interval_ms) {
    if (HAS_RGB) {
        mutex_enter_blocking(&rgb_conf.rgb_mutex);
    
        if (enable) {
            if(!rgb_conf.blink_active || rgb_conf.blink_interval != interval_ms) {
                //(Re)start timer if not active or interval changed
                if(rgb_conf.blink_active) cancel_repeating_timer(&blink_timer);
            
                rgb_conf.blink_active = true;
                rgb_conf.blink_interval = interval_ms;
                add_repeating_timer_ms(-(int32_t)interval_ms, blink_callback, NULL, &blink_timer);
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
static void destroy_system_status_struct(system_status* sys_status) {
    if (sys_status != NULL) {
        free(sys_status);
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
 * @param[out] sensor_struct write was successful or not.
 */
static void init_sensor_data_physical_struct(sensor_data_physical* sensor_struct) {
    if (sensor_struct = NULL) {
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
 * @brief Write to the I2C port.
 * @details Retries write three times before quitting.
 * @param[out] Whether write was successful or not.
 */
static void init_servo_motion_profile_struct(servo_motion_profile* servo_profile, uint8_t pwm_pin) {
    if (servo_profile = NULL) {
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
 * @brief Write to the I2C port.
 * @details Retries write three times before quitting.
 * @param[out] sys_status write was successful or not.
 */
static void init_system_status_struct(system_status* sys_status) {
    if (sys_status = NULL) {
        sys_status = (system_status*) malloc(1 * sizeof(system_status));
    }

    sys_status->core0_loops = 0;
    sys_status->core1_loops = 0;
    sys_status->last_reset_core0 = time_us_32();
    sys_status->last_watchdog = time_us_32();
    sys_status->system_ok = true;
    sys_status->emergency_stop = false;

    mutex_init(&sys_status->status_mutex);
}

/** @} */ // end of struct_init

/** @defgroup static Static Hardware Functions
 *  @brief These functions are designed solely to work in this source file. Trying to use any of these from another source file will give a compiler error.
 *  @{
 */

/*!
 * @brief ADC data read callback.
 * @param t Timer structure.
 * @return Always returns true to continue timer.
 */
static bool adc_sample_callback(struct repeating_timer* t) {
    if (HAS_ADC) {
        atomic_fetch_or_explicit(&i2c_operation_flags, ADC_READ_FLAG, memory_order_relaxed);
    }
    
    return true;
}

/*!
 * @brief Convert raw ADS1115 value to voltage
 * @param raw 16-bit raw ADC value
 * @return Voltage in volts
 */
static float ads_voltage(uint16_t raw) {
    return (int16_t)raw * (4.096f / 32768.0); //±4.096V range
}

/*!
 * @brief Callback allowing for toggling of RGB functionality.
 * @return Always returns true to continue the timer.
 */
static bool blink_callback(struct repeating_timer *t) {
    if (HAS_RGB) {
        if (mutex_try_enter(&rgb_conf.rgb_mutex, NULL)) {
            if(rgb_conf.blink_active) {
                rgb_conf.blink_state = !rgb_conf.blink_state;
                
                if(rgb_conf.blink_state) {
                    //Restore original color
                    pwm_set_gpio_level(RGB_RED_PIN, gamma_table[(uint8_t)(rgb_conf.current_r * rgb_conf.current_brightness)]);
                    pwm_set_gpio_level(RGB_GREEN_PIN, gamma_table[(uint8_t)(rgb_conf.current_g * rgb_conf.current_brightness)]);
                    pwm_set_gpio_level(RGB_BLUE_PIN, gamma_table[(uint8_t)(rgb_conf.current_b * rgb_conf.current_brightness)]);
                } else {
                    //Turn off LEDs
                    pwm_set_gpio_level(RGB_RED_PIN, 0);
                    pwm_set_gpio_level(RGB_GREEN_PIN, 0);
                    pwm_set_gpio_level(RGB_BLUE_PIN, 0);
                }
            }
            mutex_exit(&rgb_conf.rgb_mutex);
        }
    }
    
    return true;
}

/*!
 * @brief ADS1115 data read handler (responds to physical interrupt).
 * @param gpio GPIO pin number.
 * @param events Triggering events.
 */
static void ads1115_drdy_handler(uint gpio, uint32_t events) {
    if (gpio == ADS1115_ALERT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        atomic_fetch_or_explicit(&i2c_operation_flags, ADC_READ_FLAG, memory_order_relaxed);
    }
}

/*!
 * @brief GY271 data read handler (responds to physical interrupt).
 * @param gpio GPIO pin number.
 * @param events Triggering events.
 */
static void gy271_drdy_handler(uint gpio, uint32_t events) {
    if (HAS_HMC5883L) {
        if(gpio == 10 && (events & GPIO_IRQ_EDGE_RISE)) {
            atomic_fetch_or_explicit(&i2c_operation_flags, HMC_READ_FLAG, memory_order_relaxed);
        }
    }
}

/*!
 * @brief MPU6050 data read handler (responds to physical interrupt).
 * @param gpio GPIO pin number.
 * @param events Triggering events.
 */
static void mpu6050_drdy_handler(uint gpio, uint32_t events) {
    if (gpio == MPU6050_INT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        atomic_fetch_or_explicit(&i2c_operation_flags, MPU_READ_FLAG, memory_order_relaxed);
    }
}

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
 * @brief System heartbeat callback.
 * @param t Pointer to repeating timer structure.
 * @return Always returns true to continue timer.
 * @details Toggles onboard LED to indicate system liveliness.
 */
static bool heartbeat_callback(struct repeating_timer* t) {
    static bool led_state = false;
    
    #if defined(PICO_BOARD_IS_PICO_W)
    cyw43_arch_gpio_put(ROBOHAND_LED_PIN, led_state);
    #else
    gpio_put(ROBOHAND_LED_PIN, led_state);
    #endif
    
    led_state = !led_state;
    return true;
}

/*!
 * @brief Write to the I2C port.
 * @details Retries write three times before quitting.
 * @return Whether write was successful or not.
 */
static bool i2c_write_with_retry(uint8_t addr, const uint8_t* src, size_t len) {
    if (HAS_I2C) {
        int retries = 3;
        while(retries--) {
            if(i2c_write_blocking(I2C_PORT, addr, src, len, false) == len) 
                return true;
            sleep_ms(1);
        }

        return false;
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
 * @brief Initialize the system configuration.
 * @details Configures PWM frequency to 50Hz (20ms period) for standard servos.
 * @pre The user has the appropriate macros configured for desired system operation.
 * @post The hardware is set-up according to specifications.
 */
void init_system_comms() {
    //Hardware initialization
    if (HAS_I2C) {
        if (DEBUG) {
            printf("Initializing I2C.\r\n");
        }

        i2c_init(I2C_PORT, 400000);
        gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(SDA_PIN);
        gpio_pull_up(SCL_PIN);
        mutex_init(&i2c_mutex);

        if (DEBUG) {
            printf("I2C init complete.\r\n");
        }
    }

    if (HAS_ADC) {
        if (DEBUG) {
            printf("Initializing ADCs.\r\n");
        }

        if (HAS_PI_ADC) {
            adc_init();
            adc_gpio_init(ADC2_PIN);
        }

        if (USE_INTERRUPTS) {
            //Configure ALERT pin
            gpio_init(ADS1115_ALERT_PIN);
            gpio_set_dir(ADS1115_ALERT_PIN, GPIO_IN);
            gpio_pull_up(ADS1115_ALERT_PIN);
            gpio_set_irq_enabled_with_callback(ADS1115_ALERT_PIN, GPIO_IRQ_EDGE_FALL, true, &ads1115_drdy_handler);

            //Start first conversion
            uint8_t channel = 0;
            uint16_t config = ADS1115_BASE_CONFIG | (0x4000 | (channel << 12));
            uint8_t config_bytes[2] = {config >> 8, config & 0xFF};
            i2c_write_blocking(I2C_PORT, ADS1115_ADDR, config_bytes, 2, true);
        }

        if (USE_FALLBACK) {
            add_repeating_timer_ms(-100, adc_sample_callback, NULL, &adc_timer); //10Hz - 100ms
        }

        if (DEBUG) {
            printf("ADC init complete.\r\n");
        }
    }

    if (HAS_SERVOS) {
        if (DEBUG) {
            printf("Initializing Servos.\r\n");
        }

        init_servo_pwm();
        mutex_init(&servo_mutex);

        if (DEBUG) {
            printf("Servo init complete.\r\n");
        }
    }

    if (HAS_HMC5883L) {
        if (DEBUG) {
            printf("Configuring HMC5883L.\r\n");
        }

        uint8_t hmc_config[] = {
            HMC5883L_CONFIG_A, 
            0x70, // 8 samples averaged, 15Hz, normal mode
            HMC5883L_CONFIG_B, 
            0x20, // Gain=1.3Ga (1090 LSB/Gauss)
            HMC5883L_MODE, 
            HMC5883L_MODE_CONTINUOUS
        };

        if (USE_INTERRUPTS) {
            i2c_write_blocking(i2c1, HMC5883L_ADDR, hmc_config, 6, false);

            //Interrupt setup - Assuming pin will be held low when data ready, could this be confirmed?
            gpio_init(10);
            gpio_set_dir(10, GPIO_IN);
            gpio_pull_up(10);
            gpio_set_irq_enabled_with_callback(10, GPIO_IRQ_EDGE_FALL, true, &gy271_drdy_handler);
        }

        if (USE_FALLBACK) {
            add_repeating_timer_ms(-50, gy271_callback, NULL, &mpu_timer); //20Hz - 50ms
        }

        if (DEBUG) {
            printf("HMC5883L configuration successful.\r\n");
        }
    }

    if (HAS_MPU6050) {
        if (DEBUG) {
            printf("Configuring MPU6050.\r\n");
        }

        uint8_t mpu_init[] = {0x6B, 0x00};
        i2c_write_blocking(i2c1, MPU6050_ADDR, mpu_init, 2, false);

        if (USE_INTERRUPTS) {
            //Enable Data Ready Interrupt
            uint8_t int_enable[] = {0x38, 0x01}; // INT_ENABLE register
            i2c_write_blocking(i2c1, MPU6050_ADDR, int_enable, 2, false);

            //Configure GPIO interrupt
            gpio_init(MPU6050_INT_PIN);
            gpio_set_dir(MPU6050_INT_PIN, GPIO_IN);
            gpio_pull_up(MPU6050_INT_PIN);
            gpio_set_irq_enabled_with_callback(MPU6050_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &mpu6050_drdy_handler);
        }

        if (USE_FALLBACK) {
            add_repeating_timer_ms(-50, mpu6050_callback, NULL, &mpu_timer); //20Hz - 50ms
        }

        if (DEBUG) {
            printf("Configuring MPU6050.\r\n");
        }

    }
}

/*!
 * @brief Initializes system watchdog.
 * @details System has to reset watchdog within 30s window.
 * @post If the system does not reset the watchdog within the time frame, the system will reset.
 */
static void init_watchdog(void) {
    watchdog_enable(30* 1000, 1); //30s timeout
    //irq_set_enabled(WATCHDOG_IRQ, false);
}

/*!
 * @brief ADS1115 and internal ADC data read callback.
 * @param t Timer structure.
 * @return true if the callback should continue running.
 */
static bool adc_callback(struct repeating_timer* t) {
    if (HAS_ADC) {
        atomic_fetch_or_explicit(&i2c_operation_flags, ADC_READ_FLAG, memory_order_relaxed);
        return true;
    }
    
    else {
        return false;
    }
}

/*!
 * @brief GY271 data read callback.
 * @param t Timer structure.
 * @return true if the callback should continue running.
 */
static bool gy271_callback(struct repeating_timer* t) {
    if (HAS_HMC5883L) {
        atomic_fetch_or_explicit(&i2c_operation_flags, HMC_READ_FLAG, memory_order_relaxed);
        return true;
    }

    else {
        return false;
    }
}


/*!
 * @brief MPU6050 data read callback.
 * @param t Timer structure.
 * @return true if the callback should continue running.
 */
static bool mpu6050_callback(struct repeating_timer* t) {
    if (HAS_MPU6050) {
        atomic_fetch_or_explicit(&i2c_operation_flags, MPU_READ_FLAG, memory_order_relaxed);
        return true;
    }

    else {
        return false;
    }
}

/*!
 * @brief Read Pico's internal ADC channel 2.
 * @details needed because ADS only has 4 inputs.
 * @return Voltage in volts.
 */
static float read_adc2(void) {
    if (HAS_PI_ADC) {
        adc_select_input(2);
        return adc_read() * 3.3f / 4096.0f;
    }

    return -4.096f;
}

/*!
 * @brief Reads all ADC's and stores the resultant data in the sensor_readings struct.
 * @post sensor_readings contains updated ADC data.
 */
static void read_adc_data(void) {
    static uint8_t current_channel = 0;

    if (HAS_ADS1115) {
        uint16_t raw = read_ads_channel(current_channel);
        float voltage = ads_voltage(raw);
        
        mutex_enter_blocking(&sensor_readings.data_mutex);
        sensor_readings.adc_values[current_channel] = voltage;
        mutex_exit(&sensor_readings.data_mutex);

        current_channel = (current_channel + 1) % 4;
        
        // Start next conversion
        uint16_t config = ADS1115_BASE_CONFIG | (0x4000 | (current_channel << 12));
        uint8_t config_bytes[2] = {config >> 8, config & 0xFF};
        i2c_write_blocking(I2C_PORT, ADS1115_ADDR, config_bytes, 2, true);
    }
    // Read Pico's ADC2 (existing code)
    if (HAS_PI_ADC) {
        float adc2_val = read_adc2();
        mutex_enter_blocking(&sensor_readings.data_mutex);
        sensor_readings.adc_values[4] = adc2_val;
        mutex_exit(&sensor_readings.data_mutex);
    }
}

/*!
 * @brief Read specified channel from ADS1115 ADC.
 * @param channel ADC channel to read (0-3).
 * @return Raw 16-bit ADC value.
 */
static uint16_t read_ads_channel(uint8_t channel) {
    if (HAS_ADS1115) {
        uint16_t config = ADS1115_BASE_CONFIG | (0x4000 | (channel << 12)); //Set MUX
        uint8_t config_bytes[2] = {config >> 8, config & 0xFF};
        i2c_write_blocking(I2C_PORT, ADS1115_ADDR, config_bytes, 2, true);
        sleep_ms(8); //Wait for conversion
        uint8_t reg = 0x00; //Conversion register
        i2c_write_blocking(I2C_PORT, ADS1115_ADDR, &reg, 1, true);
        uint8_t buffer[2];
        i2c_read_blocking(I2C_PORT, ADS1115_ADDR, buffer, 2, false);
        return (buffer[0] << 8) | buffer[1];
    }

    else {
        return ~0;
    }
}

/*!
 * @brief Function that reads the HMC5883L data.
 * @details Uses I2C bus to get magenetic field measurements.
 * @post The sensor_readings structure contains the latest magnetic field.
 */
static void read_hmc5883l_data(void) {
    if (HAS_HMC5883L) {
        uint8_t buffer[6];
    
        if(mutex_try_enter(&i2c_mutex, NULL)) {
            i2c_write_blocking_until(I2C_PORT, HMC5883L_ADDR, (uint8_t[]){HMC5883L_DATA}, 1, true, time_us_64() + 1000);
            i2c_read_blocking_until(I2C_PORT, HMC5883L_ADDR, buffer, 6, false, time_us_64() + 2000);
            mutex_exit(&i2c_mutex);
            mutex_enter_blocking(&sensor_readings.data_mutex);
            sensor_readings.mag[0] = (buffer[0] << 8) | buffer[1];
            sensor_readings.mag[1] = (buffer[4] << 8) | buffer[5]; //HMC5883L XYZ order
            sensor_readings.mag[2] = (buffer[2] << 8) | buffer[3];
            mutex_exit(&sensor_readings.data_mutex);
        }
    }

    else {
        //Only write the invalid values once
        if (!hmc5883l_written) {
            mutex_enter_blocking(&sensor_readings.data_mutex);
            sensor_readings.mag[0] = ~0;
            sensor_readings.mag[1] = ~0;
            sensor_readings.mag[2] = ~0;
            mutex_exit(&sensor_readings.data_mutex);

            hmc5883l_written = true;
        }
    }

}

/*!
 * @brief Function that reads the MPU6050 data.
 * @details Uses the I2C bus to get linear acceleration and angular velocity measurements.
 * @post The sensor_readings structure contains the latest acceleration and rpy values.
 */
static void read_mpu6050_data(void) {
    if (HAS_MPU6050) {
        uint8_t buffer[14];
    
        if(mutex_try_enter(&i2c_mutex, NULL)) {
            i2c_write_blocking_until(I2C_PORT, MPU6050_ADDR, 
                               (uint8_t[]){MPU6050_ACCEL_XOUT_H}, 1, false, time_us_64() + 1000);
            i2c_read_blocking_until(I2C_PORT, MPU6050_ADDR, buffer, 14, false, time_us_64() + 2000);
            mutex_exit(&i2c_mutex);
            mutex_enter_blocking(&sensor_readings.data_mutex);
            sensor_readings.accel[0] = (buffer[0] << 8) | buffer[1];
            sensor_readings.accel[1] = (buffer[2] << 8) | buffer[3];
            sensor_readings.accel[2] = (buffer[4] << 8) | buffer[5];
            sensor_readings.gyro[0] = (buffer[8] << 8) | buffer[9];
            sensor_readings.gyro[1] = (buffer[10] << 8) | buffer[11];
            sensor_readings.gyro[2] = (buffer[12] << 8) | buffer[13];
            mutex_exit(&sensor_readings.data_mutex);
        }
    }

    else {
        if (!mpu6050_written) {
            mutex_enter_blocking(&sensor_readings.data_mutex);
            sensor_readings.accel[0] = ~0;
            sensor_readings.accel[1] = ~0;
            sensor_readings.accel[2] = ~0;
            sensor_readings.gyro[0] = ~0;
            sensor_readings.gyro[1] = ~0;
            sensor_readings.gyro[2] = ~0;
            mutex_exit(&sensor_readings.data_mutex);

            mpu6050_written = true;
        }
    }
}

/*!
 * @brief Update servo positions using motion profiles.
 * @details Calculates smooth transitions between current and target positions.
 * @post Servo PWM outputs are updated with new calculated positions.
 */
static void update_servo_positions(void) {
    if (HAS_SERVOS) {
        absolute_time_t now = get_absolute_time();
        int32_t dt_us = absolute_time_diff_us(prev_update_time, now);
    
        if(dt_us <= 0) return;  //Prevent time travel
    
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
                    servo_profiles[i].current_pw += (int32_t)(delta * eased_t);
                    servo_profiles[i].current_pw = constrain(
                        servo_profiles[i].current_pw, SERVO_MIN_PULSE, SERVO_MAX_PULSE
                    );
                }

                //Update PWM (50Hz = 20ms period)
                uint slice = pwm_gpio_to_slice_num(SERVO_PINS[i]);
                uint16_t wrap = pwm_hw->slice[slice].top;
                uint16_t level = (uint16_t)(servo_profiles[i].current_pw * PWM_US_TO_LEVEL);
                pwm_set_gpio_level(SERVO_PINS[i], level);
            
                mutex_exit(&servo_mutex);
            }
        }

        prev_update_time = now;
    }
}

/** @} */ // end of static
