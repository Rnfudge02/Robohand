/*!
 * \file Robohand_common.h
 * \brief Common macros needed by multiple files.
 * \details Try to keep code isolated to various files.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#ifndef ROBOHAND_COMMON_H
#define ROBOHAND_COMMON_H

//Ensure C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

#define HAS_PI_ADC false                                        ///< Whether a pressure sensor is connected to the pi pico
#define HAS_ADC false                                           ///< Whether or not ADC initalization is required

#define ADC2_PIN 28                                             ///< GPIO pin for Pico's ADC channel 2
#define NUM_SERVOS 5                                            ///< Number of servos to control
#define DEBUG 1                                                 ///< Enable debug output, higher levels increase verbosity

#define SYS_CLOCK 125000000                                     ///< System operating frequency

#define NUM_PRESPNTS 5                                          ///< Number of pressure points to sample

#ifdef __cplusplus
}
#endif

#endif