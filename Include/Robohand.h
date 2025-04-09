/*!
 * \file Robohand.h
 * \brief Robotic hand control hardware interface.
 * \details Used for other robohand modules, which each handle different methods of connection.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#ifndef ROBOHAND_H
#define ROBOHAND_H

//Ensure C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

//Includes
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/watchdog.h"
#include "hardware/regs/watchdog.h"

//Allows the system to use the correct built-in LED
#if defined(PICO_BOARD_pico_w) || defined(PICO_BOARD_IS_PICO_W)

#include "pico/cyw43_arch.h"
#define ROBOHAND_LED_PIN CYW43_WL_GPIO_LED_PIN

#else

#define ROBOHAND_LED_PIN 25                                     ///< Pin to use for heartbeat callback   

#endif

#ifndef WATCHDOG_IRQ

#define WATCHDOG_IRQ 1                                          ///< Needed for watchdog intervention

#endif

//System configuration
#define DEBUG 1                                                 ///< Enable debug output (0 - Disabled, 1 - Enabled)

#define HAS_ADS1115 false                                       ///< Whether the ADS1115 (ADC) is connected to the I2C bus
#define HAS_BME280 true                                         ///< Whether the BME280 altitude sensor is connected to the I2C bus
#define HAS_QMC5883L true                                       ///< Whether the QMC5883L (Magnometer) is connected to the i2c bus
#define HAS_MPU6050 true                                        ///< Whether the MPU6050 (Accelerometer) is connected to the i2c bus
#define HAS_I2C (HAS_ADS1115 || HAS_BME280 || HAS_QMC5883L || HAS_MPU6050)    ///< Whether or not I2C initialization is required
#define HAS_PI_ADC false                                        ///< Whether a pressure sensor is connected to the pi pico
#define HAS_ADC (HAS_PI_ADC || HAS_ADS1115)                     ///< Whether or not ADC initalization is required
#define HAS_RGB true                                            ///< Whether common Cathode RGB LED is connected to the pi pico
#define HAS_SERVOS false                                        ///< Whether servos are connected to the device

#define USE_INTERRUPTS false                                    ///< Use interupts for system communication
#define USE_DMA false                                           ///< Use DMA for system communication
#define USE_CALLBACKS (!(USE_INTERRUPTS || USE_DMA))             ///< Use fallback if other options are not selected

#define SYS_CLOCK 125000000                                     ///< System operating frequency
#define I2C_PORT i2c0                                           ///< I2C port used for device connections(i2c0 on pico)
#define I2C_DMA_TX DREQ_I2C0_TX                                 ///< 
#define I2C_DMA_RX DREQ_I2C0_RX                                 ///< 

#define NUM_SERVOS 5                                            ///< Number of servos to control
#define NUM_PRESPNTS 5                                          ///< Number of pressure points to sample
#define DMA_IN_USE 4                                            ///< Number of channels to use for DMA

#define SDA_PIN 0                                               ///< GPIO pin for I2C SDA
#define SCL_PIN 1                                               ///< GPIO pin for I2C SCL
#define ADC2_PIN 28                                             ///< GPIO pin for Pico's ADC channel 2
#define ADS1115_INT_PIN 21                                      ///< Data ready pin for the ADS1115
#define BME280_INT_PIN 20                                       ///< Data ready pin for the BME280
#define MPU6050_INT_PIN 26                                      ///< Data ready pin for the MPU6050 
#define QMC5883L_INT_PIN 22                                     ///< Data ready pin for the QMC5883L
#define RGB_RED_PIN 18                                          ///< GPIO pin connected to the red channel
#define RGB_GREEN_PIN 17                                        ///< GPIO pin connected to the green channel
#define RGB_BLUE_PIN 16                                         ///< GPIO pin connected to the blue channel

#define SERVO_MIN_PULSE 500                                     ///< Lower threshold for pulse time
#define SERVO_MAX_PULSE 2500                                    ///< Upper threshold for pulse time
#define MAX_MOVE_DURATION_MS 15000                              ///< Max amount of time the motor is allowed to move over before timeout
#define SERVO_PWM_FREQ 50                                       ///< Desired frequency for PWM response
#define CLK_DIV 64.f                                            ///< Clock divisor used for PWM
#define WRAP_VAL ((SYS_CLOCK / (SERVO_PWM_FREQ * CLK_DIV)) - 1) ///< 125MHz divided by the clock divider and the desired frequency. Will loop from 0 - calculated value

#define MAX_SERVO_ACCEL 2500                                    ///< µs/s² (adjust for servo dynamics)

//I2C Addresses
#define ADS1115_ADDR 0x48                                       ///< I2C address of ADS1115 ADC
#define BME280_ADDR 0x76                                        ///< I2C address of BME280 pressure sensor
#define MPU6050_ADDR 0x68                                       ///< I2C address of MPU6050 IMU
#define QMC5883L_ADDR 0x0D                                      ///< I2C address of QMC5883L magnetometer



//QMC5883L Registers
#define QMC5883L_CONFIG_A 0x00                                  ///< Configuration register A
#define QMC5883L_CONFIG_B 0x01                                  ///< Configuration register B
#define QMC5883L_MODE 0x02                                      ///< Mode register
#define QMC5883L_DATA 0x03                                      ///< Data output register

#define QMC5883L_MODE_CONTINUOUS 0x00                           ///< Continious sampling mode

//ADS1115 Configuration Macros
#define ADS1115_OS_SINGLE   0x8000                              ///< Start single-conversion
#define ADS1115_MUX_AIN0    0x4000                              ///< AIN0 vs GND
#define ADS1115_MUX_AIN1    0x5000                              ///< AIN1 vs GND
#define ADS1115_MUX_AIN2    0x6000                              ///< AIN2 vs GND
#define ADS1115_MUX_AIN3    0x7000                              ///< AIN3 vs GND
#define ADS1115_FSR_4V096   0x0200                              ///< ±4.096V range
#define ADS1115_MODE_SINGLE 0x0100                              ///< Single-shot mode
#define ADS1115_DR_128SPS   0x0080                              ///< 128 samples/sec
#define ADS1115_COMP_MODE   0x0000                              ///< Traditional comparator
#define ADS1115_COMP_POL    0x0000                              ///< Active low
#define ADS1115_COMP_LAT    0x0000                              ///< Non-latching
#define ADS1115_COMP_QUE    0x0003                              ///< Disable comparator

#define ADS1115_BASE_CONFIG (ADS1115_OS_SINGLE | ADS1115_FSR_4V096 | ADS1115_MODE_SINGLE | \
                            ADS1115_DR_128SPS | ADS1115_COMP_MODE | ADS1115_COMP_POL | \
                            ADS1115_COMP_LAT | ADS1115_COMP_QUE)                                    ///< ADS1115 base configuration

//MPU6050 Registers
#define MPU6050_ACCEL_XOUT_H 0x3B                               ///< Accelerometer data register

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
    float pressure;                                         ///< Air pressure
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
    float altitude;                                         ///< Altitude estimation
    mutex_t data_mutex;                                     ///< Mutex for thread-safe data access
    uint8_t __p10[3];                                       ///< Mutex is struct containing io_rw_32 and bool, maintain 32-bit alignment
} sensor_data_physical;

/*!
 * @brief Servo motion profile structure.
 * @details Stores parameters for smooth servo movement using trapezoidal profiles.
 * @note Maintains explicit 
 */
typedef struct {
    uint8_t pin;                                            ///< Pin to use for PWM modulation
    bool is_moving;                                         ///< Movement status flag
    uint16_t current_pw;                                    ///< Current pulse width (µs)
    uint16_t target_pw;                                     ///< Target pulse width (µs)
    uint16_t duration_ms;                                   ///< Total movement duration (ms)
    uint32_t start_time;                                    ///< Movement start timestamp (µs)
    mutex_t profile_mutex;                                  ///< Mutex for safe access
    uint8_t __p11[3];                                        ///< Mutex is struct containing io_rw_32 and bool, maintain 32-bit alignment
} servo_motion_profile;


/*!
 * @brief Status data structure.
 * @details Contains information about system health.
 * @pre acquire mutex before performing any read/write on this structure.
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

//Global variables
extern const uint8_t gamma_table[256];                      ///< Gamma table for brightness scaling
extern servo_motion_profile servo_profiles[NUM_SERVOS];     ///< Data structure array for holding servo profiles
extern system_status sys_status;                            ///< System status data structure instance
extern const uint SERVO_PINS[NUM_SERVOS];                   ///< Servo control pins
extern const float VOLTAGE_DIVIDER_RATIO;                   ///< Voltage divider ratio (1:1)

/** @defgroup user_facing User Facing Functions
 *  @brief Functions designed to be used in other source code. Allows for hardware interaction.
 *  @{
 */

/*!
 * @brief Actuates a servo to a specified position over a given duration.
 * @param servo Servo index (0 to NUM_SERVOS-1).
 * @param pulse_width Target pulse width in microseconds (500-2500µs).
 * @param duration_ms Movement duration in milliseconds.
 */
void actuate_servo(uint8_t servo, uint16_t pulse_width, uint16_t duration_ms);

/*!
 * @brief Actuates a servo to a specified position over a given duration.
 * @param servo Servo index (0 to NUM_SERVOS-1).
 * @param pulse_width Target pulse width in microseconds (500-2500µs).
 * @param duration_ms Movement duration in milliseconds.
 */
uint16_t constrain_u16(uint16_t value, uint16_t min, uint16_t max);

/*!
 * @brief Converts passed sensor data to physical parameters.
 * @param[in] raw Pointer to sensor_data structure to containing unconverted values.
 * @param[out] converted Pointer to sensor_data_physical structure to receive readings as physical parameters.
 * @return true if data copied successfully, false if mutex was busy.
 * @warning Caller must allocate destination buffer. Data valid until next update.
 */
bool convert_sensor_data(const sensor_data* raw, sensor_data_physical* converted);

/*!
 * @brief Core 1 main execution loop.
 * @details Handles all hardware-related operations:
 *          - Sensor polling (accelerometer, gyroscope, magnetometer)
 *          - ADC sampling
 *          - Servo control
 *          - System status monitoring
 * @pre The system has been initialized.
 * @note Runs indefinitely after system initialization.
 */
void core1_entry(void);

/*!
 * \brief Retrieves current sensor data in a thread-safe manner.
 * \param[out] dest Pointer to sensor_data structure to receive readings.
 * \return true if data copied successfully, false if mutex was busy.
 * \pre dest points to a valid sensor_data object.
 * \warning Caller must allocate destination buffer. Data valid until next update.
 */
bool get_sensor_data(sensor_data* dest);

/*!
 * \brief Retrieves the current status of a servo.
 * \param[in] servo Servo index (0 to NUM_SERVOS-1).
 * \param[out] dest Pointer to servo_motion_profile to populate.
 * \return True if successfully acquired, false if error.
 * \note An error in this case could simply be that the mutexes could not be acquired
 */
bool get_servo_status(uint8_t servo, servo_motion_profile* dest);

/*!
 * \brief Retrieves the current status of the system.
 * \param[out] dest Pointer to system_status structure to populate.
 */
void get_system_status(system_status* dest);

/*!
 * \brief Initializes the RGB LED subsystem.
 * \details Configures PWM hardware and initializes mutexes.
 */
void init_rgb(void);

/*!
 * \brief Initializes the robotic hand system and launches Core 1.
 * \details Performs critical system initialization including:
 *          - Mutex initialization for shared data
 *          - Core 1 launch for hardware I/O operations
 * \pre This should be run from core 0.
 * \post Core 1 handles sensor polling, servo control, and system monitoring.
 */
void init_robohand_system(void);

/*!
 * \brief Configures the RGB to blink at a specified interval.
 * \details Uses a repeating timer callback to achieve blinking functionality.
 * \pre This should be run from core 0.
 * \post Core 1 handles sensor polling, servo control, and system monitoring.
 */
void rgb_blink(bool enable, uint32_t interval_ms);

/*!
 * \brief Initializes the robotic hand system and launches Core 1.
 * \details Performs critical system initialization including:
 *          - Mutex initialization for shared data
 *          - Core 1 launch for hardware I/O operations
 * \pre This should be run from core 0.
 * \post Core 1 handles sensor polling, servo control, and system monitoring.
 */
void rgb_set_brightness(float brightness); // 0.0-1.0

/*!
 * \brief Sets the RGB LED color.
 * \param[in] r Red component (0-255).
 * \param[in] g Green component (0-255).
 * \param[in] b Blue component (0-255).
 */
void rgb_set_color(uint8_t r, uint8_t g, uint8_t b);

/** @} */ // end of user_facing

#ifdef __cplusplus
}
#endif

#endif //ROBOHAND_H