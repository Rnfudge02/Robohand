
/*!
 * \file Robohand_servos.h
 * \brief Provides servo functionality for user control.
 * \details Utilizes PWM, may upgrade to DMA in the future to reduce latency.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#ifndef ROBOHAND_SERVOS_H
#define ROBOHAND_SERVOS_H

//Ensure C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "pico/mutex.h"

#include "Robohand_common.h"
#include "Robohand_struct.h"

#define HAS_SERVOS false                                        ///< Whether servos are connected to the device

#define SERVO_MIN_PULSE 500                                     ///< Lower threshold for pulse time
#define SERVO_MAX_PULSE 2500                                    ///< Upper threshold for pulse time
#define MAX_MOVE_DURATION_MS 15000                              ///< Max amount of time the motor is allowed to move over before timeout
#define SERVO_PWM_FREQ 50                                       ///< Desired frequency for PWM response
#define CLK_DIV 64.f                                            ///< Clock divisor used for PWM
#define WRAP_VAL ((SYS_CLOCK / (SERVO_PWM_FREQ * CLK_DIV)) - 1) ///< 125MHz divided by the clock divider and the desired frequency. Will loop from 0 - calculated value

#define MAX_SERVO_ACCEL 2500                                    ///< µs/s² (adjust for servo dynamics)

extern const uint SERVO_PINS[NUM_SERVOS];

extern mutex_t servo_mutex;

/** @defgroup servo_control Servo Control Functions
 *  @brief Functions for controlling servo motors.
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
 * @brief Constrains a 16-bit value between minimum and maximum bounds.
 * @param value The value to constrain.
 * @param min The minimum allowed value.
 * @param max The maximum allowed value.
 * @return The constrained value.
 */
uint16_t constrain_u16(uint16_t value, uint16_t min, uint16_t max);

/*!
 * @brief Retrieves the current status of a servo.
 * @param[in] servo Servo index (0 to NUM_SERVOS-1).
 * @param[out] dest Pointer to servo_motion_profile to populate.
 * @return True if successfully acquired, false if error.
 * @note An error in this case could simply be that the mutexes could not be acquired
 */
bool get_servo_status(uint8_t servo, servo_motion_profile* dest);

void handle_servo_commands(uint32_t cmd);
void update_servo_positions(void);
void init_servo_pwm(void);

/** @} */ // end of servo_control

#ifdef __cplusplus
}
#endif

#endif