/*!
 * \file Robohand_callbacks.h
 * \brief Simple, straightforward callback backend.
 * \details Uses repeating timers to coortidnate device read access.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#ifndef ROBOHAND_CALLBACKS_H
#define ROBOHAND_CALLBACKS_H

//Ensure C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

#define USE_CALLBACKS true                                      ///< Use interupts for system communication

/**
 * @defgroup callbacks Timer Callback Functions
 * @brief Functions for timer-based sensor polling
 * @{
 */
void init_callbacks(void);
/** @} */

#ifdef __cplusplus
}
#endif

#endif // ROBOHAND_CALLBACKS_H