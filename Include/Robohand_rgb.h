/*!
 * \file Robohand_rgb.h
 * \brief Interface for using Common Cathode LED for user feedback.
 * \details Ensure initialization is complete before using interactors.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#ifndef ROBOHAND_RGB_H
#define ROBOHAND_RGB_H

//Ensure C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define HAS_RGB true                                            ///< Whether common Cathode RGB LED is connected to the pi pico

#define RGB_RED_PIN 18                                          ///< GPIO pin connected to the red channel
#define RGB_GREEN_PIN 17                                        ///< GPIO pin connected to the green channel
#define RGB_BLUE_PIN 16                                         ///< GPIO pin connected to the blue channel

/** @defgroup rgb_control RGB LED Control Functions
 *  @brief Functions for controlling RGB LED.
 *  @{
 */

/*!
 * @brief Initializes the RGB LED subsystem.
 * @details Configures PWM hardware and initializes mutexes.
 */
void init_rgb(void);

/*!
 * @brief Sets the RGB LED color.
 * @param[in] r Red component (0-255).
 * @param[in] g Green component (0-255).
 * @param[in] b Blue component (0-255).
 */
void rgb_set_color(uint8_t r, uint8_t g, uint8_t b);

/*!
 * @brief Sets the brightness of RGB LED.
 * @param brightness Brightness level (0.0-1.0).
 */
void rgb_set_brightness(float brightness);

/*!
 * @brief Configures the RGB to blink at a specified interval.
 * @param enable Enable or disable blinking.
 * @param interval_ms Blink interval in milliseconds.
 */
void rgb_blink(bool enable, uint32_t interval_ms);

/** @} */ // end of rgb_control

#ifdef __cplusplus
}
#endif

#endif