// dma_config.h
#ifndef ROBOHAND_ADVANCED_H
#define ROBOHAND_ADVANCED_H

#define USE_DMA

#ifdef USE_DMA
#include "hardware/dma.h"
#include "hardware/irq.h"

#define DMA_CHANNEL_I2C 0
#define DMA_CHANNEL_ADC 1
#define ADC_SAMPLES 256

void dma_init(void);
void dma_i2c_read(uint8_t addr, uint8_t reg, uint8_t* buffer, size_t len);
void dma_i2c_write(uint8_t addr, const uint8_t* data, size_t len);
void dma_adc_setup(void);

#endif

#endif