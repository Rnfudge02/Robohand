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
#define MPU_READ_FLAG (1 << 0)          ///< Bit signalling an MPU read is needed over the I2C bus
#define HMC_READ_FLAG (1 << 1)          ///< Bit signalling an HMC read is needed over the I2C bus
#define ADC_READ_FLAG (1 << 2)          ///< Bit signalling an ADC read is needed over the I2C bus

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

void robohand_init_components();
void robohand_read();

#endif