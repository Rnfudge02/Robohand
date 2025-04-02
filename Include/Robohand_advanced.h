// dma_config.h
#ifndef ROBOHAND_ADVANCED_H
#define ROBOHAND_ADVANCED_H

#include "pico/stdlib.h"

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
 * @brief Structure for stroring system DMA configuration.
 * @details Stores state of all DMA channels
 */
typedef struct {
    dma_channel_state i2c_tx;
    dma_channel_state i2c_rx;
    dma_channel_state adc;
    dma_channel_state pwm;
} dma_control;

volatile uint8_t i2c_operation_flags = 0;                    //! Register coordinating I2C accesses, may change at any point

void dma_init(void);
void dma_i2c_read(uint8_t addr, uint8_t reg, uint8_t* buffer, size_t len);
void dma_i2c_write(uint8_t addr, const uint8_t* data, size_t len);
void dma_adc_setup(void);

//Interrupt handlers
void ads1115_drdy_handler(uint gpio, uint32_t events);
void gy271_drdy_handler(uint gpio, uint32_t events);
void mpu6050_drdy_handler(uint gpio, uint32_t events);

//Callbacks
bool adc_sample_callback(struct repeating_timer* t);
bool blink_callback(struct repeating_timer* t);
bool gy271_callback(struct repeating_timer* t);
bool heartbeat_callback(struct repeating_timer* t);
bool mpu6050_callback(struct repeating_timer* t);

#endif