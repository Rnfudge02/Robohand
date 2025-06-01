/*!
 * \file Robohand_dma.c
 * \brief Robohand DMA backend, very complicated, extremely performant.
 * \details Work in progress featureset.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#include "Robohand.h"
#include "Robohand_dma.h"
#include "Robohand_i2c.h"
#include "Robohand_reader.h"
#include "Robohand_struct.h"

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"

#include <stdio.h>
#include <stdlib.h>

typedef int channel_id;

/** 
 * @brief Buffer for storing MPU6050 accelerometer and gyroscope data
 */
static uint8_t mpu_buffer[14];

/**
 * @brief Buffer for storing ADC samples
 */
static uint16_t adc_buffer[DMA_ADC_SAMPLES];

/**
 * @brief DMA channel for PWM operations
 */
static int pwm_dma_chan;

/**
 * @brief DMA channel for ADC operations
 */
static int adc_dma_chan;

/**
 * @brief DMA channel for I2C transmit operations
 */
static int i2c_tx_dma_chan;

/**
 * @brief DMA channel for I2C receive operations
 */
static int i2c_rx_dma_chan;

/**
 * @brief DMA control structure for managing different DMA channels
 */
static dma_control_t dma_control;

/**
 * @defgroup dma DMA Handler Functions
 * @brief Functions to initialize and manage DMA operations
 * @{
 */
static void process_adc_samples(const uint16_t* samples);
static void __isr dma_adc_handler(void);
static bool read_mpu6050_data_dma(void);
static void start_adc_dma_sampling(void);
static uint16_t read_ads1115_dma(int channel);
/** @} */

/**
 * @brief Initialize DMA channels for different peripherals
 */
void init_dma(void) {
    // Claim DMA channels correctly
    dma_control.i2c_tx.channel = dma_claim_unused_channel(true);
    dma_control.i2c_rx.channel = dma_claim_unused_channel(true);
    dma_control.adc.channel = dma_claim_unused_channel(true);
    dma_control.pwm.channel = dma_claim_unused_channel(true);
    
    // Initialize completion flags
    atomic_store(&dma_control.i2c_tx.complete, true);
    atomic_store(&dma_control.i2c_rx.complete, true);
    atomic_store(&dma_control.adc.complete, true);
    atomic_store(&dma_control.pwm.complete, true);
    
    if (DEBUG > 0) {
        printf("DMA channels assigned: TX=%d, RX=%d, ADC=%d, PWM=%d\r\n", 
            dma_control.i2c_tx.channel, dma_control.i2c_rx.channel, 
            dma_control.adc.channel, dma_control.pwm.channel);
    }
    
    // Configure I2C TX DMA channel
    dma_channel_config tx_config = dma_channel_get_default_config(dma_control.i2c_tx.channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_dreq(&tx_config, i2c_get_dreq(I2C_PORT, true));
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);
    dma_channel_configure(
        dma_control.i2c_tx.channel,
        &tx_config,
        &i2c_get_hw(I2C_PORT)->data_cmd, // Write to I2C data register
        NULL,  // Source address (set later)
        0,     // Transfer count (set later)
        false  // Don't start yet
    );
    
    // Configure I2C RX DMA channel
    dma_channel_config rx_config = dma_channel_get_default_config(dma_control.i2c_rx.channel);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_dreq(&rx_config, i2c_get_dreq(I2C_PORT, false));
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, true);
    dma_channel_configure(
        dma_control.i2c_rx.channel,
        &rx_config,
        NULL,  // Destination address (set later)
        &i2c_get_hw(I2C_PORT)->data_cmd, // Read from I2C data register
        0,     // Transfer count (set later)
        false  // Don't start yet
    );
    
    // Configure ADC DMA if needed
    if (HAS_PI_ADC) {
        adc_fifo_setup(true, true, 1, false, false); // Enable FIFO, threshold 1, no error bit
        
        dma_channel_config adc_config = dma_channel_get_default_config(dma_control.adc.channel);
        channel_config_set_transfer_data_size(&adc_config, DMA_SIZE_16);
        channel_config_set_read_increment(&adc_config, false);
        channel_config_set_write_increment(&adc_config, true);
        channel_config_set_dreq(&adc_config, DREQ_ADC);
        
        // Set up IRQ handler
        dma_channel_set_irq0_enabled(dma_control.adc.channel, true);
        irq_set_exclusive_handler(DMA_IRQ_0, dma_adc_handler);
        irq_set_enabled(DMA_IRQ_0, true);
        
        dma_channel_configure(
            dma_control.adc.channel,
            &adc_config,
            adc_buffer,      // Destination buffer
            &adc_hw->fifo,   // Source: ADC FIFO
            DMA_ADC_SAMPLES, // Number of samples
            false            // Don't start yet
        );
    }
}

/**
 * @brief Interrupt handler for ADC DMA completion
 */
static void __isr dma_adc_handler(void) {
    // Clear the interrupt request for the ADC channel
    if (dma_channel_get_irq0_status(dma_control.adc.channel)) {
        dma_channel_acknowledge_irq0(dma_control.adc.channel);
        
        // Signal completion
        dma_control.adc.complete = true;
        
        // Process the samples
        process_adc_samples(adc_buffer);
    }
}

/**
 * @brief Process the DMA-acquired ADC samples
 * 
 * @param samples Pointer to buffer containing the ADC samples
 */
static void process_adc_samples(const uint16_t *samples) {
    if (!samples) return;
    
    uint32_t sum = 0;
    
    // Average all samples
    for (int i = 0; i < DMA_ADC_SAMPLES; i++) {
        sum += samples[i];
    }
    
    float average = (float)sum / DMA_ADC_SAMPLES;
    float voltage = average * 3.3f / 4096.0f; // 12-bit ADC with 3.3V reference
    
    // Update sensor data thread-safely
    if (mutex_enter_timeout_ms(&sensor_readings.data_mutex, 50)) {
        sensor_readings.adc_values[4] = voltage;
        mutex_exit(&sensor_readings.data_mutex);
    }
    
    if (DEBUG > 1) {
        printf("ADC DMA sampling complete: avg=%f, voltage=%.3fV\r\n", average, voltage);
    }
}

void read_adc_data(void) {
    static int current_channel = 0;

    if (HAS_ADS1115) {
        int raw = ~0;
        if (USE_DMA) {
            raw = read_ads1115_dma(current_channel);
        }
        
        else {
           raw = read_ads_channel(current_channel);
        }
        
        float voltage = ads_voltage((uint16_t) abs(raw));
        
        mutex_enter_blocking(&sensor_readings.data_mutex);
        sensor_readings.adc_values[current_channel] = voltage;
        mutex_exit(&sensor_readings.data_mutex);

        current_channel = (current_channel + 1) % 4;
        
        // Start next conversion
        uint16_t config = (uint16_t) (ADS1115_BASE_CONFIG | (0x4000 | (current_channel << 12)));
        uint8_t config_bytes[2] = {config >> 8, config & 0xFF};
        i2c_write_with_retry(ADS1115_ADDR, config_bytes, 2, true, 2000);
    }

    // Read Pico's ADC2
    if (HAS_PI_ADC) {
        adc_select_input(2);
        float adc2_val = adc_read() * 3.3f / 4096.0f;
        mutex_enter_blocking(&sensor_readings.data_mutex);
        sensor_readings.adc_values[4] = adc2_val;
        mutex_exit(&sensor_readings.data_mutex);
    }
}

/**
 * @brief Read MPU6050 data using DMA transfer
 * 
 * @return true if read was successful, false otherwise
 */
static bool read_mpu6050_data_dma(void) {
    if (!HAS_MPU6050) return false;
    
    if (!mutex_try_enter(&i2c_mutex, NULL)) {
        return false;
    }
    
    // Check if DMA is already in progress
    if (!atomic_load(&dma_control.i2c_tx.complete) || !atomic_load(&dma_control.i2c_rx.complete)) {
        mutex_exit(&i2c_mutex);
        return false;
    }
    
    // Prepare the register address (0x3B - ACCEL_XOUT_H)
    static uint8_t reg_addr = MPU6050_ACCEL_XOUT_H;
    
    // Mark DMA as in use
    atomic_store(&dma_control.i2c_tx.complete, false);
    atomic_store(&dma_control.i2c_rx.complete, false);
    
    // Configure TX DMA to write register address
    dma_channel_set_read_addr(dma_control.i2c_tx.channel, &reg_addr, 1);
    dma_channel_set_trans_count(dma_control.i2c_tx.channel, 1, true);
    
    // Wait for TX to complete
    dma_channel_wait_for_finish_blocking(dma_control.i2c_tx.channel);
    
    // Configure RX DMA to read data
    dma_channel_set_write_addr(dma_control.i2c_rx.channel, mpu_buffer, 14);
    dma_channel_set_trans_count(dma_control.i2c_rx.channel, 14, true);
    
    // Wait for RX to complete
    dma_channel_wait_for_finish_blocking(dma_control.i2c_rx.channel);
    
    // Mark DMA as complete
    atomic_store(&dma_control.i2c_tx.complete, true);
    atomic_store(&dma_control.i2c_rx.complete, true);
    
    // Clear the I2C interrupt
    uint8_t int_status_reg = 0x3A;
    uint8_t int_status;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &int_status_reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &int_status, 1, false);
    
    mutex_exit(&i2c_mutex);
    
    // Process the data
    if (mutex_enter_timeout_ms(&sensor_readings.data_mutex, 50)) {
        // Process accelerometer data (bytes 0-5)
        sensor_readings.accel[0] = (int16_t)((mpu_buffer[0] << 8) | mpu_buffer[1]);
        sensor_readings.accel[1] = (int16_t)((mpu_buffer[2] << 8) | mpu_buffer[3]);
        sensor_readings.accel[2] = (int16_t)((mpu_buffer[4] << 8) | mpu_buffer[5]);
        
        // Process gyroscope data (bytes 8-13)
        sensor_readings.gyro[0] = (int16_t)((mpu_buffer[8] << 8) | mpu_buffer[9]);
        sensor_readings.gyro[1] = (int16_t)((mpu_buffer[10] << 8) | mpu_buffer[11]);
        sensor_readings.gyro[2] = (int16_t)((mpu_buffer[12] << 8) | mpu_buffer[13]);
        
        mutex_exit(&sensor_readings.data_mutex);
        return true;
    }
    
    return false;
}

/**
 * @brief Start ADC sampling using DMA transfer
 */
static void start_adc_dma_sampling(void) {
    if (!HAS_PI_ADC || !USE_DMA) return;
    
    // Check if DMA is already in progress
    if (!dma_control.adc.complete) {
        return;
    }
    
    // Mark DMA as in use
    dma_control.adc.complete = false;
    
    // Select ADC input
    adc_select_input(2); // ADC2 - channel configured as ADC2_PIN
    
    // Start the ADC in free-running mode
    adc_run(true);
    
    // Start the DMA transfer
    dma_channel_start(dma_control.adc.channel);
}

/**
 * @brief Read ADS1115 channel using DMA transfer
 * 
 * @param channel ADS1115 channel to read (0-3)
 * @return Raw 16-bit ADC value
 */
/**
 * @brief Read ADS1115 channel using DMA transfer
 * 
 * @param channel ADS1115 channel to read (0-3)
 * @return Raw 16-bit ADC value
 */
static uint16_t read_ads1115_dma(int channel) {
    if (!HAS_ADS1115 || !USE_DMA) {
        return 0xFFFF;
    }
    
    if (!mutex_try_enter(&i2c_mutex, NULL)) {
        return 0xFFFF;
    }
    
    // Check if DMA is already in progress
    if (!atomic_load(&dma_control.i2c_tx.complete) || !atomic_load(&dma_control.i2c_rx.complete)) {
        mutex_exit(&i2c_mutex);
        return 0xFFFF;
    }
    
    // Configure ADS1115 for the selected channel
    uint16_t config = (uint16_t) (ADS1115_BASE_CONFIG | (0x4000 | (channel << 12)));
    uint8_t config_bytes[3] = {0x01, (uint8_t) (config >> 8), (uint8_t) (config & 0xFF)};
    
    // Mark DMA as in use
    atomic_store(&dma_control.i2c_tx.complete, false);
    
    // Configure and start TX DMA to write config register
    dma_channel_set_read_addr(dma_control.i2c_tx.channel, config_bytes, 3);
    dma_channel_set_trans_count(dma_control.i2c_tx.channel, 3, true);
    
    // Wait for TX to complete
    dma_channel_wait_for_finish_blocking(dma_control.i2c_tx.channel);
    atomic_store(&dma_control.i2c_tx.complete, true);
    
    // Conversion time (worst case ~8ms for 128SPS)
    sleep_ms(10);
    
    // Prepare to read conversion register (0x00)
    uint8_t conv_reg = 0x00;
    atomic_store(&dma_control.i2c_tx.complete, false);
    
    // Configure and start TX DMA to write conversion register address
    dma_channel_set_read_addr(dma_control.i2c_tx.channel, &conv_reg, 1);
    dma_channel_set_trans_count(dma_control.i2c_tx.channel, 1, true);
    
    // Wait for TX to complete
    dma_channel_wait_for_finish_blocking(dma_control.i2c_tx.channel);
    atomic_store(&dma_control.i2c_tx.complete, true);
    
    // Read the conversion result
    uint8_t result[2];
    atomic_store(&dma_control.i2c_rx.complete, false);
    
    // Configure and start RX DMA to read conversion result
    dma_channel_set_write_addr(dma_control.i2c_rx.channel, result, 2);
    dma_channel_set_trans_count(dma_control.i2c_rx.channel, 2, true);
    
    // Wait for RX to complete
    dma_channel_wait_for_finish_blocking(dma_control.i2c_rx.channel);
    atomic_store(&dma_control.i2c_rx.complete, true);
    
    mutex_exit(&i2c_mutex);
    
    // Combine the bytes into a 16-bit result
    return (uint16_t) ((result[0] << 8) | result[1]);
}