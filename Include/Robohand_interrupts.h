/*!
 * \file Robohand_interrupts.h
 * \brief Interrupt backend, for balance of power with complexity.
 * \details Still a work-in-progress.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#ifndef ROBOHAND_INTERRUPTS_H
#define ROBOHAND_INTERRUPTS_H

//Ensure C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "pico/stdlib.h"

#define USE_INTERRUPTS false                                    ///< Use interupts for system communication

#define ADS1115_INT_PIN 22                                      ///< Data ready pin for the ADS1115
#define BME280_INT_PIN 21                                       ///< Data ready pin for the BME280
#define MPU6050_INT_PIN 20                                      ///< Data ready pin for the MPU6050 
#define QMC5883L_INT_PIN 19                                     ///< Data ready pin for the QMC5883L

/**
 * @defgroup interrupts Interrupt Handler Functions
 * @brief Functions to initialize and handle hardware interrupts
 * @{
 */
void init_interrupts(void);
static void ads1115_drdy_handler(uint gpio, uint32_t events);
static void bme280_drdy_handler(uint gpio, uint32_t events);
static void gy271_drdy_handler(uint gpio, uint32_t events);
static void mpu6050_drdy_handler(uint gpio, uint32_t events);
/** @} */

#ifdef __cplusplus
}
#endif

#endif // ROBOHAND_INTERRUPTS_H