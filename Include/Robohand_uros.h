/*!
 * \file Robohand_uros.h
 * \brief Micro-ROS interface for robotic hand control system.
 * \details Provides declarations for micro-ROS integration with sensor data publishing
 *          and servo command subscription. Manages custom serial transport implementation
 *          for Pico-RTOS compatibility.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

/*! \todo Fix issues present within code, check for correctness.
 *  \todo Ensure code for preventing the running of code on missing components does not break functionality.
 *  \todo Fix issue with HMC5883L always returning 0. (observed behavior when running program with USB driver)
 */

#ifndef ROBOHAND_UROS_H
#define ROBOHAND_UROS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/types.h>

//Pico and ROS includes
#include "pico/stdlib.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>
 
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rmw_microros/rmw_microros.h>

#define SENSOR_DATA_COUNT 14
#define SENSOR_PUB_INTERVAL_MS 1000

/*! @defgroup debug_macros MicroROS Debugging Macros
 *  @brief Macros to provide extra stability to the application. Prints error message if referenced function returns an error.
 *  @{
 */

#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK) && DEBUG) { \
        printf("[%s:%d] Failed: %d\n", __FILE__, __LINE__, (int)temp_rc); \
    } \
} ///< Checks return code from RCL functions. Logs failures with file/line info when DEBUG is enabled.

#define RCSOFTCHECK(fn) RCCHECK(fn) ///< Soft error checking macro for non-critical operations.

/** @} */ // end of debug_macros

/*! @defgroup time_utils Time Compatibility Layer
 *  @brief POSIX time functions for micro-ROS compatibility.
 *  @{
 */

/*!
 * @brief Provides POSIX-compatible clock implementation.
 * @param[in] clk_id Clock identifier (unused).
 * @param[out] tp Timespec structure to populate.
 * @return Always returns 0 (success).
 * @note Uses Pico's time_us_64() for timing.
 */
int clock_gettime(clockid_t clk_id, struct timespec* tp);

/*!
 * @brief Provides POSIX-compatible sleep implementation.
 * @param us Microsecond count to sleep.
 */
void usleep(uint64_t us);

/** @} */ // end of time_utils

/*! @defgroup transport_functions Custom Transport Implementation
 *  @brief Low-level serial communication functions for micro-ROS.
 *  @{
 */

/*!
 * @brief Initializes custom serial transport for micro-ROS.
 * @param[in] transport Pointer to custom transport structure.
 * @return true if initialization succeeded, false otherwise.
 * @note Configures standard I/O only once during first call.
 */
bool pico_serial_transport_open(struct uxrCustomTransport* transport);

/*!
 * @brief Closes custom serial transport.
 * @param[in] transport Pointer to custom transport structure.
 * @return Always returns true (Pico serial doesn't require close handling).
 */
bool pico_serial_transport_close(struct uxrCustomTransport* transport);

/*!
 * @brief Writes data through serial transport.
 * @param[in] transport Pointer to custom transport structure.
 * @param[in] buf Buffer containing data to write.
 * @param[in] len Length of data to write.
 * @param[out] errcode Error code output (1 on failure).
 * @return Number of bytes actually written.
 */
size_t pico_serial_transport_write(struct uxrCustomTransport* transport, 
                                   const uint8_t* buf, 
                                   size_t len, 
                                   uint8_t* errcode);

/*!
 * @brief Reads data from serial transport.
 * @param[in] transport Pointer to custom transport structure.
 * @param[out] buf Buffer to store read data.
 * @param[in] len Maximum length to read.
 * @param[in] timeout Timeout in milliseconds.
 * @param[out] errcode Error code output (1 on failure).
 * @return Number of bytes actually read.
 */
size_t pico_serial_transport_read(struct uxrCustomTransport* transport,
                                  uint8_t* buf,
                                  size_t len,
                                  int timeout,
                                  uint8_t* errcode);

 /** @} */ // end of transport_functions

/** @defgroup ros_interface ROS Message Handling.
 *  @brief Functions for managing ROS communication and data conversion.
 *  @{
 */

/*!
 * @brief Callback for servo command messages.
 * @param[in] msg_in Received Int32MultiArray message pointer.
 * @details Expects message data in triplets: [servo_num, target_pw, duration_ms].
 * @note Servo numbers are validated against NUM_SERVOS constant.
 */
void servo_callback(const void* msg_in);

/*!
 * @brief Timer callback for sensor data publishing.
 * @param[in] timer Timer object reference.
 * @param[in] last_call_time Timestamp of last invocation (unused).
 * @details Publishes combined sensor data at 10Hz rate:
 *          - Accelerometer (3 axes, m/s²)
 *          - Gyroscope (3 axes, rad/s)
 *          - Magnetometer (3 axes, μT)
 *          - Analog sensors (5 channels, volts)
 */
void sensor_timer_callback(rcl_timer_t* timer, int64_t last_call_time);

 /** @} */ // ros_interface_functions

#ifdef __cplusplus
}
#endif

#endif //ROBOHAND_UROS_H