/*!
 * \file Robohand_init.h
 * \brief Initializes connected peripherals to well-known state.
 * \details Change register values to modify configuration.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#ifndef ROBOHAND_INIT_H
#define ROBOHAND_INIT_H

//Ensure C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @defgroup initialization Hardware Initialization Functions
 * @brief Functions to initialize various hardware components
 * @{
 */

/*!
 * @brief Initializes all hardware components based on configuration.
 * @details Sets up I2C, sensors, and communication backends.
 * @return True if all enabled components initialized successfully, false otherwise.
 */
bool robohand_init_components(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif