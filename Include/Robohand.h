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

//Global variables
extern const float VOLTAGE_DIVIDER_RATIO;                   ///< Voltage divider ratio (1:1)

/** @defgroup system_init System Initialization Functions
 *  @brief Functions for initializing the robotic hand system.
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
void init_robohand_system(void);



/** @} */ // end of system_init

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
 * @brief Gets debug information about the system.
 * @details Prints out detailed system status and diagnostics to serial output.
 */
void get_debug_info(void);

/** @} */ // end of system_status

/** @defgroup sensor_operations Sensor Operations
 *  @brief Functions for sensor reading and processing.
 *  @{
 */

/*!
 * @brief Reads sensor data based on operation flags.
 * @details Handles all sensor read operations as triggered by flags.
 */
void robohand_read(void);

/*!
 * @brief Retrieves and prints diagnostic information.
 * @details Displays I2C status, sensor readings, and system health.
 */
void retrieve_debug_info(void);

/** @} */ // end of sensor_operations

#ifdef __cplusplus
}
#endif

#endif //ROBOHAND_H