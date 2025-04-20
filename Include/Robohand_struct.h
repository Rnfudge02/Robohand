/*!
 * \file Robohand_struct.h
 * \brief Contains functions for interacting with system structures.
 * \details Will set clearly invalid values on init (planned).
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#ifndef ROBOHAND_STRUCT_H
#define ROBOHAND_STRUCT_H

//Ensure C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

#include <stdatomic.h>                                          ///< Needed for atomic instructions
#include <stdbool.h>                                            ///< For bool struct members
#include <stdint.h>                                             ///< For uintX_t struct members

#include "pico/mutex.h"                                         ///< For Mutex struct members

#include "Robohand_common.h"

/** @defgroup dma_structures DMA Structures
 *  @brief Structures for managing DMA operations.
 *  @{
 */
 
/*!
 * @brief Structure for storing the data for a DMA channel.
 * @details Contains the channel ID, whether the request is complete, and its IRQ status.
 */
typedef struct {
    int channel;                                                ///< DMA channel number
    atomic_bool complete;                                       ///< Flag indicating if transfer is complete
    uint32_t irq_status;                                        ///< IRQ status of the channel
} dma_channel_state;

/*!
 * @brief Structure for storing system DMA configuration.
 * @details Stores state of all DMA channels
 */
typedef struct {
    dma_channel_state i2c_tx;                                   ///< I2C transmit DMA channel state
    dma_channel_state i2c_rx;                                   ///< I2C receive DMA channel state
    dma_channel_state adc;                                      ///< ADC DMA channel state
    dma_channel_state pwm;                                      ///< PWM DMA channel state
} dma_control_t;

/** @} */ // end of dma_structures

/** @defgroup system_structs System Structures
 *  @brief Structures used during program execution. Eases development with safe, lockable access to resources.
 *  @note Mutexes contain spinlock (type io_rw_32) and bool (uint8_t), to be safe insert 3-bytes padding at the end.
 *  @{
 */

/*!
 * @brief Structure for RGB LED configuration and state.
 * @details Stores current color, brightness, and synchronization primitives.
 */
typedef struct {
    bool blink_active;                                      ///< Whether blinking is active
    bool blink_state;                                       ///< Current state of the blink (on/off)
    uint8_t current_r;                                      ///< Current red value (0-255)
    uint8_t current_g;                                      ///< Current green value (0-255)
    uint8_t current_b;                                      ///< Current blue value (0-255)
    uint8_t __p0;                                           ///< Maintaining alignment for next uint16_t access
    uint16_t pwm_wrap;                                      ///< PWM wrap value for frequency control
    uint32_t blink_interval;                                ///< Blink interval in milliseconds
    float current_brightness;                               ///< Current brightness (0.0-1.0)
    mutex_t rgb_mutex;                                      ///< Mutex for color/brightness access
    uint8_t __p1[3];                                        ///< Mutex is struct containing io_rw_32 and bool, maintain 32-bit alignment
    mutex_t pwm_mutex;                                      ///< Mutex for PWM hardware access, unaligned access to io_rw_32 will cause a fault
    uint8_t __p2[3];                                        ///< Mutex is struct containing io_rw_32 and bool, maintain 32-bit alignment
} rgb_state;

/*!
 * @brief Sensor data structure with mutex protection.
 * @details Contains sensor readings from accelerometer, gyroscope, magnetometer, and ADCs.
 * @pre Acquire mutex before performing any read/write on this structure.
 */
typedef struct {
    int16_t accel[3];                                       ///< Accelerometer readings (X, Y, Z)
    uint8_t __p3[2];                                        ///< Padding for explicit 32-bit alignment (DMA)
    int16_t gyro[3];                                        ///< Gyroscope readings (X, Y, Z)
    uint8_t __p4[2];                                        ///< Padding for explicit 32-bit alignment (DMA)
    int16_t mag[3];                                         ///< Magnetometer readings (X, Y, Z)
    uint8_t __p5[2];                                        ///< Padding for explicit 32-bit alignment
    float adc_values[5];                                    ///< ADC readings (channels 0-3 + internal ADC2)
    float pressure;                                         ///< Air pressure in Pascals
    float altitude;                                         ///< Altitude in meters
    float temperature;                                      ///< Temperature in degrees Celsius
    float humidity;                                         ///< Humidity percentage (0-100%)
    mutex_t data_mutex;                                     ///< Mutex for thread-safe data access
    uint8_t __p6[3];                                        ///< Mutex is struct containing io_rw_32 and bool, maintain 32-bit alignment
} sensor_data;

/*!
 * @brief Physical sensor data structure.
 * @details Contains converted sensor readings in physical units.
 * @note Doesn't need to be used on core 1, technically doesn't need mutex, also not involved with any callbacks.
 */
typedef struct {
    float accel[3];                                         ///< Accelerometer in g (9.81 m/s²)
    float gyro[3];                                          ///< Gyroscope in degrees per second (dps)
    float mag[3];                                           ///< Magnetometer in microteslas (µT)
    float adc_values[5];                                    ///< ADC readings in volts or converted units
    float altitude;                                         ///< Altitude estimation in meters
    mutex_t data_mutex;                                     ///< Mutex for thread-safe data access
    uint8_t __p10[3];                                       ///< Mutex is struct containing io_rw_32 and bool, maintain 32-bit alignment
} sensor_data_physical;

/*!
 * @brief Servo motion profile structure.
 * @details Stores parameters for smooth servo movement using trapezoidal profiles.
 * @note Maintains explicit alignment for 32-bit boundaries
 */
typedef struct {
    uint8_t pin;                                            ///< Pin to use for PWM modulation
    bool is_moving;                                         ///< Movement status flag
    uint16_t current_pw;                                    ///< Current pulse width (µs)
    uint16_t target_pw;                                     ///< Target pulse width (µs)
    uint16_t duration_ms;                                   ///< Total movement duration (ms)
    uint32_t start_time;                                    ///< Movement start timestamp (µs)
    mutex_t profile_mutex;                                  ///< Mutex for safe access
    uint8_t __p11[3];                                       ///< Mutex is struct containing io_rw_32 and bool, maintain 32-bit alignment
} servo_motion_profile;

/*!
 * @brief Status data structure.
 * @details Contains information about system health.
 * @pre Acquire mutex before performing any read/write on this structure.
 */
typedef struct {
    uint32_t core0_loops;                                   ///< Counts on core 0 main loop since last status retrieval
    uint32_t core1_loops;                                   ///< Counts on core 1 main loop since last status retrieval
    uint32_t last_update;                                   ///< Last time the counter was reset on core 0
    float core0_load;                                       ///< Approximation of load on core 0
    float core1_load;                                       ///< Approximation of load on core 1
    uint32_t last_watchdog;                                 ///< Time since last watchdog event
    bool system_ok;                                         ///< Did the watchdog reset the system?
    bool emergency_stop;                                    ///< To be used for automatic servo stoppage
    uint8_t __p12[2];                                       ///< Make sure io_rw_32 access is on 32-bit boundary
    mutex_t status_mutex;                                   ///< Mutex to protect access to internal members
    uint8_t __p13[3];                                       ///< Mutex is struct containing io_rw_32 and bool, maintain 32-bit alignment
} system_status;

/** @} */ // end of system_structs

 /** @defgroup sensor_structures Sensor-Specific Structures
  *  @brief Structures specific to sensor operations.
  *  @{
  */
 
 /*!
  * @brief Structure to hold BME280 calibration data.
  * @details Contains all calibration coefficients needed for temperature, pressure, and humidity.
  */
 typedef struct {
    uint16_t dig_T1;                       ///< Temperature calibration T1
    int16_t dig_T2;                        ///< Temperature calibration T2
    int16_t dig_T3;                        ///< Temperature calibration T3
    uint16_t dig_P1;                       ///< Pressure calibration P1
    int16_t dig_P2;                        ///< Pressure calibration P2
    int16_t dig_P3;                        ///< Pressure calibration P3
    int16_t dig_P4;                        ///< Pressure calibration P4
    int16_t dig_P5;                        ///< Pressure calibration P5
    int16_t dig_P6;                        ///< Pressure calibration P6
    int16_t dig_P7;                        ///< Pressure calibration P7
    int16_t dig_P8;                        ///< Pressure calibration P8
    int16_t dig_P9;                        ///< Pressure calibration P9
    uint8_t dig_H1;                        ///< Humidity calibration H1
    int16_t dig_H2;                        ///< Humidity calibration H2
    uint8_t dig_H3;                        ///< Humidity calibration H3
    int16_t dig_H4;                        ///< Humidity calibration H4
    int16_t dig_H5;                        ///< Humidity calibration H5
    int8_t dig_H6;                         ///< Humidity calibration H6
} bme280_calib_data;

/** @} */ // end of sensor_structures

extern bme280_calib_data bme280_calib;
extern rgb_state rgb_conf;
extern sensor_data sensor_readings;
extern servo_motion_profile servo_profiles[NUM_SERVOS];     ///< Data structure array for holding servo profiles
extern system_status sys_status;                            ///< System status data structure instance

/** @defgroup system_status System Status Functions
 *  @brief Functions for retrieving system status information.
 *  @{
 */

/*!
 * @brief Retrieves the current status of the system.
 * @param[out] dest Pointer to system_status structure to populate.
 */
void get_system_status(system_status* dest);

/** @defgroup sensor_data Sensor Data Functions
 *  @brief Functions for retrieving and processing sensor data.
 *  @{
 */

/*!
 * @brief Retrieves current sensor data in a thread-safe manner.
 * @param[out] dest Pointer to sensor_data structure to receive readings.
 * @return true if data copied successfully, false if mutex was busy.
 * @pre dest points to a valid sensor_data object.
 * @warning Caller must allocate destination buffer. Data valid until next update.
 */
bool get_sensor_data(sensor_data* dest);

/*!
 * @brief Converts passed sensor data to physical parameters.
 * @param[in] raw Pointer to sensor_data structure to containing unconverted values.
 * @param[out] converted Pointer to sensor_data_physical structure to receive readings as physical parameters.
 * @return true if data copied successfully, false if mutex was busy.
 * @warning Caller must allocate destination buffer. Data valid until next update.
 */
bool convert_sensor_data(const sensor_data* raw, sensor_data_physical* converted);

/** @} */ // end of sensor_data

void destroy_rgb_state_struct(rgb_state* rgb_struct);
void destroy_sensor_data_struct(sensor_data* sensor_struct);
void destroy_sensor_data_physical_struct(sensor_data_physical* sensor_struct);
void destroy_servo_motion_profile_struct(servo_motion_profile* servo_profile);
void destroy_system_status_struct(system_status* sys_status);

void init_rgb_state_struct(rgb_state* rgb_struct);
void init_sensor_data_struct(sensor_data* sensor_struct);
void init_sensor_data_physical_struct(sensor_data_physical* sensor_struct);
void init_servo_motion_profile_struct(servo_motion_profile* servo_profile, uint8_t pwm_pin);
void init_system_status_struct(system_status* sys_status);

#ifdef __cplusplus
}
#endif

#endif // ROBOHAND_STRUCT_H