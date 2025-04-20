/*!
* @file Robohand.c
* @brief Implementation of robotic hand control system with dual-core operation.
* @details Manages hardware interfaces, sensor data collection, and servo control on Core 1.
* @author Robert Fudge
* @date 2025
* @copyright Apache 2.0 License
*/

#include "Robohand.h"
#include "Robohand_dma.h"
#include "Robohand_i2c.h"
#include "Robohand_init.h"
#include "Robohand_interrupts.h"
#include "Robohand_reader.h"
#include "Robohand_servos.h"
#include "Robohand_struct.h"
 
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
 
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/regs/watchdog.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"

/*******************************************************************************
 * Constants and Global Variables
 ******************************************************************************/

const float VOLTAGE_DIVIDER_RATIO = 1.0f;  ///< Ratio for voltage calculations

/** 
 * @brief Timer for heartbeat callback (Default 1000ms / 1Hz)
 */
static struct repeating_timer hb_timer;

/*******************************************************************************
 * Forward Function Declarations
 ******************************************************************************/

/** @defgroup system_init System Initialization
 *  @brief Functions for system initialization.
 *  @{
 */
static void init_watchdog(void);
static bool watchdog_callback(struct repeating_timer *t);
void estop(void);
bool core1_init(void);
void core1_handle_commands(void);
void core1_entry(void);
/** @} */ // end of system_init

/** @defgroup helper_functions Helper Functions
 *  @brief Utility and support functions.
 *  @{
 */
static bool heartbeat_callback(struct repeating_timer* t);
static uint8_t constrain_u8(uint8_t value, uint8_t min, uint8_t max);
uint16_t constrain_u16(uint16_t value, uint16_t min, uint16_t max);
/** @} */ // end of helper_functions

/** @defgroup sensor_ops Sensor Operations
 *  @brief Functions for sensor reading and management.
 *  @{
 */
void robohand_read(void);
/** @} */ // end of sensor_ops

/** @defgroup debug Debug and Diagnostic
 *  @brief Functions for system diagnostics and debugging.
 *  @{
 */
static void verify_interrupt_setup(void);
void retrieve_debug_info(void);
void get_debug_info(void);
/** @} */ // end of debug

/*******************************************************************************
 * System Initialization Implementation
 ******************************************************************************/

/** @defgroup system_init_impl System Initialization Implementation 
 *  @brief Implementation of system initialization functions.
 *  @{
 */

/*!
 * @brief Initializes the robotic hand system and launches Core 1.
 * @details Performs critical system initialization including:
 *          - Mutex initialization for shared data
 *          - Core 1 launch for hardware I/O operations
 * @pre This should be run from core 0.
 * @post Core 1 handles sensor polling, servo control, and system monitoring.
 */
void init_robohand_system(void) {
    // Initialize shared data structures and mutexes for hardware resources
    init_sensor_data_struct(&sensor_readings);

    // Launch core1 with sensor handling
    multicore_launch_core1(core1_entry);

    printf("Waiting for core1 init.\r\n");

    // Wait for core1 initialization
    while(!multicore_fifo_rvalid());

    printf("Done waiting for core1 init.\r\n");

    multicore_fifo_pop_blocking();
}

/*!
 * @brief Sets up the watchdog timer with appropriate timeout.
 * @details Configures watchdog to reset system after 8 seconds of inactivity.
 * @note Creates a timer to periodically refresh the watchdog.
 */
static void init_watchdog(void) {
    // We'll configure the watchdog with a 8-second timeout
    watchdog_enable(8000, true);  // 8s timeout, pause during debug
     
    if (DEBUG > 0) {
        printf("Watchdog enabled with 8s timeout\r\n");
    }
     
    // Create a repeating timer to periodically kick the watchdog
    // This is safer than relying solely on the main loop
    static struct repeating_timer watchdog_timer;
     
    add_repeating_timer_ms(-1000, watchdog_callback, NULL, &watchdog_timer);
}

/*!
 * @brief Callback function for the watchdog timer.
 * @param t Pointer to the repeating timer structure (unused).
 * @return Always returns true to keep the timer running.
 */
static bool watchdog_callback(struct repeating_timer *t) {
    (void) t;
    watchdog_update();
    mutex_enter_blocking(&sys_status.status_mutex);
    sys_status.last_watchdog = time_us_32();
    mutex_exit(&sys_status.status_mutex);
    return true; // Keep timer running
}
 
/*!
 * @brief Emergency stop function that immediately halts all servo movement.
 * @details Sets the emergency_stop flag and stops all servo motion.
 */
void estop(void) {       
    // Immediately stop all servos
    if (HAS_SERVOS) {
        for (int i = 0; i < NUM_SERVOS; i++) {
            if (mutex_try_enter(&servo_profiles[i].profile_mutex, NULL)) {
                servo_profiles[i].is_moving = false;
                mutex_exit(&servo_profiles[i].profile_mutex);
            }
        }
    }

    mutex_enter_blocking(&sys_status.status_mutex);
    sys_status.emergency_stop = true;
    mutex_exit(&sys_status.status_mutex);
}

/*!
 * @brief Core 1 initialization function.
 * @details Initializes Core 1 components and hardware.
 * @return Boolean indicating initialization success.
 */
bool core1_init(void) {
    static bool first_run = true;

    // First attempt to initialize components
    bool init_success = robohand_init_components();
    
    if (init_success == false && DEBUG > 0) {
        printf("WARNING: Component initialization failed, will retry\r\n");
    }

    // Initialize hardware that is always needed
    if (first_run == true) {
        mutex_init(&servo_mutex);
        init_system_status_struct(&sys_status);

        // Set up heartbeat timer
        add_repeating_timer_ms(-1000, &heartbeat_callback, NULL, &hb_timer);

        // Set up watchdog
        init_watchdog();

        first_run = false;
    }

    // If servos are enabled, initialize them
    if (HAS_SERVOS == true) {
        if (DEBUG > 0) {
            printf("Initializing Servos\r\n");
        }
    
        init_servo_pwm();
    
        if (DEBUG > 0) {
            printf("Servo init complete\r\n");
        }
    }

    // Signal to core0 that initialization is complete
    multicore_fifo_push_blocking(init_success ? 1 : 0);
    
    return init_success;
}

/*!
 * @brief Core 1 command handler function.
 * @details Processes commands received from Core 0.
 */
void core1_handle_commands(void) {
    uint32_t cmd = multicore_fifo_pop_blocking();
            
    // Handle debug ping
    if (cmd == 0xDEAD) {
        multicore_fifo_push_blocking(cmd); // Echo back debug ping
    }

    // Emergency stop command
    else if (cmd == 0xE000) {
        estop();
    }

    // Resume from emergency stop
    else if (cmd == 0xE001) {
        mutex_enter_blocking(&sys_status.status_mutex);
        sys_status.emergency_stop = false;
        mutex_exit(&sys_status.status_mutex);
    }

    // Normal servo command
    else {
        handle_servo_commands(cmd);
    }
}

/** @} */ // end of system_init_impl

/*******************************************************************************
 * Core Implementation Functions
 ******************************************************************************/

/** @defgroup core_impl Core Implementation Functions
 *  @brief Functions for Core 1 operation.
 *  @{
 */

/*!
 * @brief Core 1 main entry function.
 * @details Handles sensor reading, servo updates, and command processing.
 */
void core1_entry(void) {
    uint32_t loop_count = 0;
    bool init_success = core1_init();
    
    // Main loop with retry logic for component initialization
    absolute_time_t last_load_update = get_absolute_time();
    absolute_time_t last_retry = get_absolute_time();
    
    while(1) {
        loop_count++;
        
        // If initialization failed, periodically retry (every 5 seconds)
        if (!init_success && absolute_time_diff_us(last_retry, get_absolute_time()) > 5000000) {
            init_success = robohand_init_components();
            last_retry = get_absolute_time();
            
            if (init_success && DEBUG > 0) {
                printf("Component initialization succeeded on retry\r\n");
            }
        }
        
        // Update load metrics every second
        if (absolute_time_diff_us(last_load_update, get_absolute_time()) > 1000000) {
            mutex_enter_blocking(&sys_status.status_mutex);
            sys_status.core1_loops = loop_count;
            last_load_update = get_absolute_time();
            mutex_exit(&sys_status.status_mutex);
            loop_count = 0;
        }
        
        // Check for commands from core 0
        if (multicore_fifo_rvalid()) {
            core1_handle_commands();
        }
        
        // Only read sensors and update servos if not in emergency stop
        bool emergency_stop = false;
        mutex_enter_blocking(&sys_status.status_mutex);
        emergency_stop = sys_status.emergency_stop;
        mutex_exit(&sys_status.status_mutex);
        
        if (!emergency_stop) {
            // Read sensor data
            robohand_read();
            
            // Update servo positions
            if (HAS_SERVOS) {
                update_servo_positions();
            }
        }
        
        // Always update the watchdog
        watchdog_update();
        
        // Sleep to prevent tight-looping
        sleep_ms(50);
    }
}

/** @} */ // end of core_impl

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

/** @defgroup helper_functions_impl Helper Functions Implementation
 *  @brief Implementation of utility and support functions.
 *  @{
 */

/*!
 * @brief Constrains a 16-bit value between minimum and maximum bounds.
 * @param value The value to constrain.
 * @param min The minimum allowed value.
 * @param max The maximum allowed value.
 * @return The constrained value.
 */
uint16_t constrain_u16(uint16_t value, uint16_t min, uint16_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/*!
 * @brief Constrains an 8-bit value between minimum and maximum bounds.
 * @param value The value to constrain.
 * @param min The minimum allowed value.
 * @param max The maximum allowed value.
 * @return The constrained value.
 */
static uint8_t constrain_u8(uint8_t value, uint8_t min, uint8_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/*!
 * @brief System heartbeat callback.
 * @param t Pointer to repeating timer structure.
 * @return Always returns true to continue timer.
 * @details Toggles onboard LED to indicate system liveliness.
 */
static bool heartbeat_callback(struct repeating_timer* t) {
    (void)t;

    static bool led_state = false;
    
    // Needs to be macro, because regular Pico target doesn't have these functions
    #if defined(PICO_BOARD_IS_PICO_W)
    cyw43_arch_gpio_put(ROBOHAND_LED_PIN, led_state);
    #else
    gpio_put(ROBOHAND_LED_PIN, led_state);
    #endif
    
    led_state = !led_state;
    return true;
}

/** @} */ // end of helper_functions_impl

/*******************************************************************************
 * Sensor Reading Implementation
 ******************************************************************************/

/** @defgroup sensor_ops_impl Sensor Operations Implementation
 *  @brief Implementation of sensor reading functions.
 *  @{
 */

/**
 * @brief Main sensor reading function, processes flags to read sensors
 * 
 * @details This function checks the operation flags and reads data from
 *          the enabled sensors based on those flags
 */
void robohand_read(void) {
    uint8_t current_flags = atomic_load(&i2c_operation_flags);

    // Handle sensor reads via flags
    if((current_flags & ADC_READ_FLAG) && HAS_ADC) {
        read_adc_data();
        // Atomic update to clear the flag
        atomic_fetch_and(&i2c_operation_flags, ~ADC_READ_FLAG);
    }

    if ((current_flags & BME_READ_FLAG) && HAS_BME280) {
        bool success = read_bme280_data();
        if (success) {
            // Only clear flag if read was successful
            atomic_fetch_and(&i2c_operation_flags, ~BME_READ_FLAG);
        }
    }

    if ((current_flags & MPU_READ_FLAG) && HAS_MPU6050) {
        bool success = read_mpu6050_data();
        
        if (success) {
            // Only clear flag if read was successful
            atomic_fetch_and(&i2c_operation_flags, ~MPU_READ_FLAG);
        }
    }
            
    if((current_flags & QMC_READ_FLAG) && HAS_QMC5883L) {
        bool success = read_qmc5883l_data();
        if (success) {
            // Only clear flag if read was successful
            atomic_fetch_and(&i2c_operation_flags, ~QMC_READ_FLAG);
        }
    }
}

/** @} */ // end of sensor_ops_impl

/*******************************************************************************
 * Debug and Diagnostic Functions
 ******************************************************************************/

/** @defgroup debug_impl Debug and Diagnostic Implementation
 *  @brief Implementation of system diagnostics and debugging functions.
 *  @{
 */

/**
 * @brief Retrieves and displays debug information about the system
 * 
 * @details Scans the I2C bus, checks interrupt pin states, displays
 *          operation flags, and forces sensor reads for debugging
 */
void retrieve_debug_info(void) {
    printf("Debug Information:\r\n");
    printf("I2C Bus Scan:\r\n");
    i2c_scan_bus();
    
    printf("Interrupt Pin States:\r\n");
    verify_interrupt_setup();
    
    printf("Operation Flags: 0x%02X\r\n", i2c_operation_flags);
    
    printf("Forcing sensor reads...\r\n");
    // Force set all flags
    i2c_operation_flags |= (ADC_READ_FLAG | BME_READ_FLAG | MPU_READ_FLAG | QMC_READ_FLAG);
    robohand_read();
    
    printf("Sensor read complete.\r\n");
}

/**
 * @brief Comprehensive debug information and diagnostics
 * 
 * @details Provides detailed system status, sensor readings, and hardware state
 */
void get_debug_info(void) {
    printf("\r\n===== Debug Information =====\r\n");
    
    printf("System Status:\r\n");
    system_status status;
    get_system_status(&status);
    
    printf("  Core0 load: %.2f%%\r\n", status.core0_load * 100.0f);
    printf("  Core1 load: %.2f%%\r\n", status.core1_load * 100.0f);
    printf("  System OK: %s\r\n", status.system_ok ? "Yes" : "No");
    printf("  Emergency stop: %s\r\n", status.emergency_stop ? "Yes" : "No");
    printf("  Last watchdog: %lu us ago\r\n", time_us_32() - status.last_watchdog);
    
    printf("\r\nI2C Bus Scan:\r\n");
    i2c_scan_bus();
    
    printf("\r\nInterrupt Pin States:\r\n");
    verify_interrupt_setup();
    
    printf("\r\nOperation Flags: 0x%02X\r\n", i2c_operation_flags);
    printf("  ADC read needed: %s\r\n", (i2c_operation_flags & ADC_READ_FLAG) ? "Yes" : "No");
    printf("  BME read needed: %s\r\n", (i2c_operation_flags & BME_READ_FLAG) ? "Yes" : "No");
    printf("  MPU read needed: %s\r\n", (i2c_operation_flags & MPU_READ_FLAG) ? "Yes" : "No");
    printf("  QMC read needed: %s\r\n", (i2c_operation_flags & QMC_READ_FLAG) ? "Yes" : "No");
    
    printf("\r\nServo Status:\r\n");
    if (HAS_SERVOS) {
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            servo_motion_profile profile;
            if (get_servo_status(i, &profile)) {
                printf("  Servo %d: %s, position=%d, target=%d, duration=%d ms\r\n",
                      i, profile.is_moving ? "moving" : "idle",
                      profile.current_pw, profile.target_pw, profile.duration_ms);
            } else {
                printf("  Servo %d: status unavailable\r\n", i);
            }
        }
    }
    else {
        printf("  Servos disabled\r\n");
    }
    
    printf("\r\nSensor Readings:\r\n");
    sensor_data data;
    if (get_sensor_data(&data)) {
        printf("  Accel: X=%d, Y=%d, Z=%d\r\n", data.accel[0], data.accel[1], data.accel[2]);
        printf("  Gyro: X=%d, Y=%d, Z=%d\r\n", data.gyro[0], data.gyro[1], data.gyro[2]);
        printf("  Mag: X=%d, Y=%d, Z=%d\r\n", data.mag[0], data.mag[1], data.mag[2]);
        printf("  ADC values: %.3fV, %.3fV, %.3fV, %.3fV, %.3fV\r\n",
              data.adc_values[0], data.adc_values[1], data.adc_values[2],
              data.adc_values[3], data.adc_values[4]);
        printf("  Pressure: %.2f Pa\r\n", data.pressure);
        printf("  Temperature: %.2f °C\r\n", data.temperature);
        printf("  Humidity: %.2f %%\r\n", data.humidity);
        printf("  Altitude: %.2f m\r\n", data.altitude);
    } else {
        printf("  Failed to get sensor data\r\n");
    }
    
    printf("============================\r\n\r\n");
}

/**
 * @brief Verify interrupt pin setup and report status
 * 
 * @details Checks and reports the state of all sensor interrupt pins
 */
static void verify_interrupt_setup(void) {
    if (HAS_MPU6050) {
        printf("MPU6050 interrupt pin %d state: %d\r\n", 
               MPU6050_INT_PIN, gpio_get(MPU6050_INT_PIN));
    }
    
    if (HAS_BME280) {
        printf("BME280 interrupt pin %d state: %d\r\n", 
               BME280_INT_PIN, gpio_get(BME280_INT_PIN));
    }
    
    if (HAS_QMC5883L) {
        printf("QMC5883L interrupt pin %d state: %d\r\n", 
               QMC5883L_INT_PIN, gpio_get(QMC5883L_INT_PIN));
    }
    
    if (HAS_ADS1115) {
        printf("ADS1115 interrupt pin %d state: %d\r\n", 
               ADS1115_INT_PIN, gpio_get(ADS1115_INT_PIN));
    }
}

/** @} */ // end of debug_impl