/*!
 * @file Robohand_uros.h
 * @brief MicroROS interface header for robotic hand control system.
 * @details Defines structures and functions for MicroROS communication.
 * @author Robert Fudge <rnfudge@mun.ca>
 * @date 2025
 * @copyright Apache 2.0 License
 */

#ifndef ROBOHAND_UROS_H
#define ROBOHAND_UROS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Robohand.h"

// MicroROS topic names
#define TOPIC_SERVO_CMD    "robohand/servo"
#define TOPIC_SENSOR_DATA  "robohand/sensors"
#define TOPIC_SYSTEM_STATUS "robohand/status"

// Custom message type definitions (if needed)
// For now, we're using standard ROS2 messages

// Function prototypes
void uros_init(void);
void uros_spin(void);
void uros_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif // ROBOHAND_UROS_H