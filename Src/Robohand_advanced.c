#include "Robohand_advanced.h"
#include "Robohand.h"

#include <stdio.h>

#include <stdatomic.h>

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

#ifdef USE_INTERRUPTS

/** @defgroup interrupt_handlers Interrupt Handlers
 *  @brief Used to eliminate polling. Change the CMake macro -DUSE_INTERRUPTS=ON to utilizes this backend.
 *  @{
 */

/*!
 * @brief ADS1115 data read handler (responds to physical interrupt).
 * @param gpio GPIO pin number.
 * @param events Triggering events.
 */
static void ads1115_drdy_handler(uint gpio, uint32_t events) {
    if (gpio == ADS1115_INT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        atomic_fetch_or_explicit(&i2c_operation_flags, ADC_READ_FLAG, memory_order_relaxed);
    }
}

/*!
 * @brief GY271 data read handler (responds to physical interrupt).
 * @param gpio GPIO pin number.
 * @param events Triggering events.
 */
static void gy271_drdy_handler(uint gpio, uint32_t events) {
    if (HAS_HMC5883L) {
        if(gpio == 10 && (events & GPIO_IRQ_EDGE_RISE)) {
            atomic_fetch_or_explicit(&i2c_operation_flags, HMC_READ_FLAG, memory_order_relaxed);
        }
    }
}

/*!
 * @brief MPU6050 data read handler (responds to physical interrupt).
 * @param gpio GPIO pin number.
 * @param events Triggering events.
 */
static void mpu6050_drdy_handler(uint gpio, uint32_t events) {
    if (gpio == MPU6050_INT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        atomic_fetch_or_explicit(&i2c_operation_flags, MPU_READ_FLAG, memory_order_relaxed);
    }
}

/** @} */ // end of interrupt_handlers

#endif

/** @defgroup callbacks Callbacks
 *  @brief Used as a fallback if no other implementations are available (USE_FALLBACK macro is true).
 *  @{
 */

 #ifdef USE_FALLBACK

 /*!
 * @brief ADC data read callback.
 * @param t Timer structure.
 * @return Always returns true to continue timer.
 */
static bool adc_sample_callback(struct repeating_timer* t) {
    if (HAS_ADC) {
        atomic_fetch_or_explicit(&i2c_operation_flags, ADC_READ_FLAG, memory_order_relaxed);
    }
    
    return true;
}

/*!
 * @brief GY271 data read callback.
 * @param t Timer structure.
 * @return true if the callback should continue running.
 */
static bool gy271_callback(struct repeating_timer* t) {
    if (HAS_HMC5883L) {
        atomic_fetch_or_explicit(&i2c_operation_flags, HMC_READ_FLAG, memory_order_relaxed);
        return true;
    }

    else {
        return false;
    }
}

/*!
 * @brief MPU6050 data read callback.
 * @param t Timer structure.
 * @return true if the callback should continue running.
 */
static bool mpu6050_callback(struct repeating_timer* t) {
    if (HAS_MPU6050) {
        atomic_fetch_or_explicit(&i2c_operation_flags, MPU_READ_FLAG, memory_order_relaxed);
        return true;
    }

    else {
        return false;
    }
}

#endif

/*!
 * @brief Callback allowing for toggling of RGB functionality.
 * @return Always returns true to continue the timer.
 */
static bool blink_callback(struct repeating_timer *t) {
    if (HAS_RGB) {
        if (mutex_try_enter(&rgb_conf.rgb_mutex, NULL)) {
            if(rgb_conf.blink_active) {
                rgb_conf.blink_state = !rgb_conf.blink_state;
                
                if(rgb_conf.blink_state) {
                    //Restore original color
                    pwm_set_gpio_level(RGB_RED_PIN, gamma_table[(uint8_t)(rgb_conf.current_r * rgb_conf.current_brightness)]);
                    pwm_set_gpio_level(RGB_GREEN_PIN, gamma_table[(uint8_t)(rgb_conf.current_g * rgb_conf.current_brightness)]);
                    pwm_set_gpio_level(RGB_BLUE_PIN, gamma_table[(uint8_t)(rgb_conf.current_b * rgb_conf.current_brightness)]);
                } else {
                    //Turn off LEDs
                    pwm_set_gpio_level(RGB_RED_PIN, 0);
                    pwm_set_gpio_level(RGB_GREEN_PIN, 0);
                    pwm_set_gpio_level(RGB_BLUE_PIN, 0);
                }
            }
            mutex_exit(&rgb_conf.rgb_mutex);
        }
    }
    
    return true;
}



/*!
 * @brief System heartbeat callback.
 * @param t Pointer to repeating timer structure.
 * @return Always returns true to continue timer.
 * @details Toggles onboard LED to indicate system liveliness.
 */
static bool heartbeat_callback(struct repeating_timer* t) {
    static bool led_state = false;
    
    #if defined(PICO_BOARD_IS_PICO_W)
    cyw43_arch_gpio_put(ROBOHAND_LED_PIN, led_state);
    #else
    gpio_put(ROBOHAND_LED_PIN, led_state);
    #endif
    
    led_state = !led_state;
    return true;
}

/** @} */ // end of callbacks