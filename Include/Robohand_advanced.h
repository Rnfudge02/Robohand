// dma_config.h
#ifndef ROBOHAND_ADVANCED_H
#define ROBOHAND_ADVANCED_H

#include "pico/stdlib.h"
#include <stdint.h>
#include "Robohand.h"

#define DMA_CHANNEL_I2C 0
#define DMA_CHANNEL_ADC 1
#define DMA_ADC_SAMPLES 256

//Macros for I2C flag
#define ADC_READ_FLAG (1 << 0)          ///< Bit signalling an ADC read is needed over the I2C bus
#define BME_READ_FLAG (1 << 1)          ///< Bit signalling a BME read is needed
#define MPU_READ_FLAG (1 << 2)          ///< Bit signalling a MPU read is needed over the I2C bus
#define QMC_READ_FLAG (1 << 3)          ///< Bit signalling a QMC read is needed over the I2C bus


/*!
 * @brief Structure for storing the data for a DMA channel.
 * @details Contains the channel ID, whether the request is complete, and its IRQ status.
 */
typedef struct {
    uint8_t channel;
    volatile bool complete;
    uint32_t irq_status;
} dma_channel_state;

/*!
 * @brief Structure for storing system DMA configuration.
 * @details Stores state of all DMA channels
 */
typedef struct {
    dma_channel_state i2c_tx;
    dma_channel_state i2c_rx;
    dma_channel_state adc;
    dma_channel_state pwm;
} dma_control;

extern sensor_data sensor_readings;                                        //! Mutex protected structure containing sensor information

#if HAS_BME280 == true

/**
 * @brief Structure to hold BME280 calibration data
 */
typedef struct {
    uint16_t dig_T1;  ///< Temperature calibration T1
    int16_t dig_T2;   ///< Temperature calibration T2
    int16_t dig_T3;   ///< Temperature calibration T3
    uint16_t dig_P1;  ///< Pressure calibration P1
    int16_t dig_P2;   ///< Pressure calibration P2
    int16_t dig_P3;   ///< Pressure calibration P3
    int16_t dig_P4;   ///< Pressure calibration P4
    int16_t dig_P5;   ///< Pressure calibration P5
    int16_t dig_P6;   ///< Pressure calibration P6
    int16_t dig_P7;   ///< Pressure calibration P7
    int16_t dig_P8;   ///< Pressure calibration P8
    int16_t dig_P9;   ///< Pressure calibration P9
    uint8_t dig_H1;   ///< Humidity calibration H1
    int16_t dig_H2;   ///< Humidity calibration H2
    uint8_t dig_H3;   ///< Humidity calibration H3
    int16_t dig_H4;   ///< Humidity calibration H4
    int16_t dig_H5;   ///< Humidity calibration H5
    int8_t dig_H6;    ///< Humidity calibration H6
} bme280_calib_data;

// Global calibration data variable
extern bme280_calib_data bme280_calib;

#endif

void robohand_init_components();
void robohand_read();

#endif