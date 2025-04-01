#include "Robohand_advanced.h"
#include "Robohand.h"

#include <stdio.h>

#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"


#ifdef USE_DMA

typedef int channel_id;

static channel_id dma_i2c_chan;
static uint8_t dma_i2c_buffer[32];
static uint8_t mpu_buffer[14]; // Accel + gyro data
static uint16_t adc_buffer[DMA_ADC_SAMPLES];
static int pwm_dma_chan;  // Track PWM DMA channel
static uint16_t adc_buffer[DMA_ADC_SAMPLES];
static int adc_dma_chan;

//DMA channels for I2C0/I2C1
static int i2c_tx_dma_chan;
static int i2c_rx_dma_chan;

void process_adc_samples(uint16_t* samples);

void __isr adc_isr() {
    adc_fifo_drain();
    process_adc_samples(adc_buffer); // Your data handler
}

void check_dma_perf() {
    uint32_t cycles = dma_hw->ch[pwm_dma_chan].al3_read_addr_trig;
    printf("PWM DMA used %lu cycles\n", cycles);
}

void init_i2c_dma() {
    i2c_tx_dma_chan = dma_claim_unused_channel(true);
    i2c_rx_dma_chan = dma_claim_unused_channel(true);
    
    // Configure TX channel (I2C writes)
    dma_channel_config c = dma_channel_get_default_config(i2c_tx_dma_chan);
    channel_config_set_dreq(&c, I2C_PORT == i2c0 ? DREQ_I2C0_TX : DREQ_I2C1_TX);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, DREQ_I2C1_TX); // For I2C1
    dma_channel_configure(i2c_tx_dma_chan, &c,
                          &i2c_get_hw(I2C_PORT)->data_cmd, // Write to I2C data register
                          NULL, // Dest addr set per-transfer
                          0,    // Count set later
                          false);

    // Configure RX channel (I2C reads)
    c = dma_channel_get_default_config(i2c_rx_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, DREQ_I2C1_RX);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    dma_channel_configure(i2c_rx_dma_chan, &c,
                          NULL, // Dest addr set per-transfer
                          &i2c_get_hw(I2C_PORT)->data_cmd,
                          0,
                          false);
}



void read_mpu6050_dma() {
    //1. Write register address (0x3B)
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    dma_channel_set_write_addr(i2c_tx_dma_chan, &reg, true);
    dma_channel_set_trans_count(i2c_tx_dma_chan, 1, true);
    
    //2. Read 14 bytes via DMA
    dma_channel_set_read_addr(i2c_rx_dma_chan, mpu_buffer, true);
    dma_channel_set_trans_count(i2c_rx_dma_chan, 14, true);
    
    //3. Chain transfers
    dma_channel_start(i2c_tx_dma_chan);
    dma_channel_wait_for_finish_blocking(i2c_tx_dma_chan);
    dma_channel_start(i2c_rx_dma_chan);
    dma_channel_wait_for_finish_blocking(i2c_rx_dma_chan);
}

uint16_t dma_read_ads_channel(uint8_t channel) {
    // DMA implementation for ADS1115
    // Similar pattern to regular code but using DMA transfers
}




void init_adc_dma() {
    adc_dma_chan = dma_claim_unused_channel(true);
    adc_fifo_setup(true, true, 1, false, false);
    
    dma_channel_config c = dma_channel_get_default_config(adc_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, (I2C_PORT == i2c0) ? DREQ_I2C0_TX : DREQ_I2C1_TX);
    
    dma_channel_configure(adc_dma_chan, &c,
                          adc_buffer,        // Dest
                          &adc_hw->fifo,     // Src
                          DMA_ADC_SAMPLES,       // Count
                          true);             // Start

    irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_isr);
    irq_set_enabled(ADC_IRQ_FIFO, true);
}

void process_adc_samples(uint16_t* samples) {
    // Your sample processing logic
}

#endif