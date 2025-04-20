/*!
 * \file Robohand_callbacks.c
 * \brief Simple, straightforward callback backend.
 * \details Uses repeating timers to coortidnate device read access.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#include "Robohand.h"
#include "Robohand_callbacks.h"
#include "Robohand_common.h"
#include "Robohand_i2c.h"

#include <stdatomic.h>

/** 
 * @brief Timer for ADC callback (Default 100ms / 10Hz)
 */
static struct repeating_timer adc_timer;

/** 
 * @brief Timer for BME280 pressure sensor (Default 100ms / 10Hz)
 */
static struct repeating_timer bme_timer;

/** 
 * @brief Timer for QMC5883L callback (Default 50ms / 20Hz)
 */
static struct repeating_timer gy271_timer;

/** 
 * @brief Timer for MPU6050 callback (Default 100ms / 10Hz)
 */
static struct repeating_timer mpu_timer;

/**
 * @defgroup callbacks Timer Callback Functions
 * @brief Functions for timer-based sensor polling
 * @{
 */
static bool adc_sample_callback(struct repeating_timer* t);
static bool bme280_callback(struct repeating_timer* t);
static bool gy271_callback(struct repeating_timer* t);
static bool mpu6050_callback(struct repeating_timer* t);
/** @} */

/**
 * @brief Initialize timer-based sensor data acquisition
 */
void init_callbacks(void) {
    if (HAS_PI_ADC || HAS_ADS1115) {
        add_repeating_timer_ms(-100, &adc_sample_callback, NULL, &adc_timer); // 10Hz - 100ms
    }
    
    if (HAS_BME280) {
        add_repeating_timer_ms(-100, &bme280_callback, NULL, &bme_timer); // 10Hz - 100ms
    }
    
    if (HAS_MPU6050) {
        add_repeating_timer_ms(-50, &mpu6050_callback, NULL, &mpu_timer); // 20Hz - 50ms
    }

    if (HAS_QMC5883L) {
        add_repeating_timer_ms(-100, &gy271_callback, NULL, &gy271_timer); // 10Hz - 100ms
    }
}

/**
 * @brief Timer callback for ADC data acquisition
 * 
 * @param t Pointer to the repeating timer structure
 * @return true to continue timer, false to stop
 */
static bool adc_sample_callback(struct repeating_timer* t) {
    (void)t;    // Cast to void to avoid static analysis errors

    if (HAS_ADC) {
        uint8_t expected = i2c_operation_flags;
        uint8_t desired = expected | ADC_READ_FLAG;
        atomic_compare_exchange_strong(&i2c_operation_flags, &expected, desired);

        return true;
    }

    else {
        return false;
    }
}

/**
 * @brief Timer callback for BME280 data acquisition
 * 
 * @param t Pointer to the repeating timer structure
 * @return true to continue timer, false to stop
 */
static bool bme280_callback(struct repeating_timer* t) {
    (void)t;    // Cast to void to avoid static analysis errors

    if (HAS_BME280) {
        uint8_t expected = i2c_operation_flags;
        uint8_t desired = expected | BME_READ_FLAG;
        atomic_compare_exchange_strong(&i2c_operation_flags, &expected, desired);

        return true;
    }

    else {
        return false;
    }
}

/**
 * @brief Timer callback for MPU6050 data acquisition
 * 
 * @param t Pointer to the repeating timer structure
 * @return true to continue timer, false to stop
 */
static bool mpu6050_callback(struct repeating_timer* t) {
    (void)t;    // Cast to void to avoid static analysis errors

    if (HAS_MPU6050) {
        uint8_t expected = i2c_operation_flags;
        uint8_t desired = expected | MPU_READ_FLAG;
        atomic_compare_exchange_strong(&i2c_operation_flags, &expected, desired);

        return true;
    }

    else {
        return false;
    }
}

/**
 * @brief Timer callback for QMC5883L data acquisition
 * 
 * @param t Pointer to the repeating timer structure
 * @return true to continue timer, false to stop
 */
static bool gy271_callback(struct repeating_timer* t) {
    (void)t;    // Cast to void to avoid static analysis errors

    if (HAS_QMC5883L) {
        uint8_t expected = i2c_operation_flags;
        uint8_t desired = expected | QMC_READ_FLAG;
        atomic_compare_exchange_strong(&i2c_operation_flags, &expected, desired);

        return true;
    }

    else {
        return false;
    }
}
