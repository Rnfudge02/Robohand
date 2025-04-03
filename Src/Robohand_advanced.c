#include "Robohand_advanced.h"

#include <stdio.h>

#include <stdatomic.h>

#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

//Interrupt handler functions
static void init_interrupts();
static void ads1115_drdy_handler(uint gpio, uint32_t events);
static void gy271_drdy_handler(uint gpio, uint32_t events);
static void mpu6050_drdy_handler(uint gpio, uint32_t events);

//Callback functions
static void init_callbacks();
static bool adc_sample_callback(struct repeating_timer* t);
static bool gy271_callback(struct repeating_timer* t);
static bool mpu6050_callback(struct repeating_timer* t);

//Reader functions
static inline uint16_t read_ads_channel(uint8_t channel);
static inline float ads_voltage(uint16_t raw);

static void read_adc_data(void);
static void read_hmc5883l_data(void);
static void read_mpu6050_data(void);

static volatile uint8_t i2c_operation_flags = 0;                    //! Register coordinating I2C accesses, may change at any point
sensor_data sensor_readings;
static mutex_t i2c_mutex;

void robohand_init_components(void) {
    if (DEBUG) {
        printf("Initializing I2C.\r\n");
    }

    i2c_init(I2C_PORT, 400000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    mutex_init(&i2c_mutex);

#if HAS_HMC5883L == true
    if (DEBUG) {
        printf("Configuring HMC5883L.\r\n");
    }

    uint8_t hmc_config[] = {
        HMC5883L_CONFIG_A, 
        0x70, // 8 samples averaged, 15Hz, normal mode
        HMC5883L_CONFIG_B, 
        0x20, // Gain=1.3Ga (1090 LSB/Gauss)
        HMC5883L_MODE, 
        HMC5883L_MODE_CONTINUOUS
    };
    i2c_write_blocking(I2C_PORT, HMC5883L_ADDR, hmc_config, 6, false);

    if (DEBUG) {
        printf("Configured HMC5883L.\r\n");
    }
#endif

#if HAS_MPU6050 == true
    if (DEBUG) {
        printf("Configuring MPU6050.\r\n");
    }

    uint8_t mpu_init[] = {
        0x6B,  // PWR_MGMT_1 register
        0x00,  // Wake up device + use internal clock
        0x1B,  // GYRO_CONFIG register
        0x08,  // ±500dps full scale
        0x1C,  // ACCEL_CONFIG register
        0x10   // ±8g full scale
    };
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, mpu_init, 6, false);

    if (DEBUG) {
        printf("Configured MPU6050.\r\n");
    }

#endif

    if (DEBUG) {
        printf("I2C initialized.\r\n");
    }

#if HAS_PI_ADC == true
    if (DEBUG) {
        printf("Initializing Pico ADCs.\r\n");
    }

    adc_init();
    adc_gpio_init(ADC2_PIN);

    if (DEBUG) {
        printf("Pico ADCs initialized.\r\n");
    }
#endif

#if USE_DMA == true
    if (DEBUG == true) {
        printf("Intitializing DMA Backend.\r\n");
    }

    init_dma();

    if (DEBUG == true) {
        printf("DMA initialized.\r\n");
    }
#endif

#if USE_INTERRUPTS == true
    if (DEBUG == true) {
        printf("Intitializing Interrupt Backend.\r\n");
    }

    init_interrupts();

    if (DEBUG == true) {
        printf("Interrupts initialized.\r\n");
    }
#endif

#if USE_FALLBACK == true
    if (DEBUG == true) {
        printf("Intitializing Callback Backend.\r\n");
    }

    init_callbacks();

    if (DEBUG == true) {
        printf("Callback initialized.\r\n");
    }
#endif
}

void robohand_read() {
    //Handle sensor reads via flags
    if((i2c_operation_flags & MPU_READ_FLAG) && HAS_I2C) {
        read_mpu6050_data();
        i2c_operation_flags &= ~MPU_READ_FLAG;
    }
            
    if((i2c_operation_flags & HMC_READ_FLAG) && HAS_I2C) {
        read_hmc5883l_data();
        i2c_operation_flags &= ~HMC_READ_FLAG;
    }
    
    if((i2c_operation_flags & ADC_READ_FLAG) && HAS_I2C) {
        read_adc_data();
        i2c_operation_flags &= ~ADC_READ_FLAG;
    }
}

#if USE_DMA == true

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

/** @defgroup interrupts Interrupts
 *  @brief Used to eliminate polling. Change the CMake macro -DUSE_INTERRUPTS=ON to utilize this backend.
 *  @{
 */

#if USE_INTERRUPTS == true

static void init_interrupts(void) {
#if HAS_ADS1115 == true
    //Configure ALERT pin
    gpio_init(ADS1115_INT_PIN);
    gpio_set_dir(ADS1115_INT_PIN, GPIO_IN);
    gpio_pull_up(ADS1115_INT_PIN);
    gpio_set_irq_enabled_with_callback(ADS1115_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &ads1115_drdy_handler);

    //Start first conversion
    uint8_t channel = 0;
    uint16_t config = ADS1115_BASE_CONFIG | (0x4000 | (channel << 12));
    uint8_t config_bytes[2] = {config >> 8, config & 0xFF};
    i2c_write_blocking(I2C_PORT, ADS1115_ADDR, config_bytes, 2, true);
#endif

#if HAS_HMC5883L == true
    //Interrupt setup - Assuming pin will be held low when data ready, could this be confirmed?
    gpio_init(HMC5883L_INT_PIN);
    gpio_set_dir(HMC5883L_INT_PIN, GPIO_IN);
    gpio_pull_up(HMC5883L_INT_PIN);
    gpio_set_irq_enabled_with_callback(HMC5883L_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &gy271_drdy_handler);
#endif

#if HAS_MPU6050 == true
    if (DEBUG) {
        printf("Configuring MPU6050.\r\n");
    }

    //Enable Data Ready Interrupt
    uint8_t int_enable[] = {0x38, 0x01}; // INT_ENABLE register
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, int_enable, 2, false);

    //Configure GPIO interrupt
    gpio_init(MPU6050_INT_PIN);
    gpio_set_dir(MPU6050_INT_PIN, GPIO_IN);
    gpio_pull_up(MPU6050_INT_PIN);
    gpio_set_irq_enabled_with_callback(MPU6050_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &mpu6050_drdy_handler);
#endif
}

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

/** @} */ // end of interrupts

#endif

#if USE_FALLBACK == true

/** @defgroup callbacks Callbacks
 *  @brief Used as a fallback if no other implementations are available (USE_FALLBACK macro is true).
 *  @{
 */

static struct repeating_timer adc_timer;                            //! Timer for ADC callback (Default 100ms / 10Hz)
static struct repeating_timer gy271_timer;                            //! Timer for ADC callback (Default 50ms / 20Hz)
static struct repeating_timer mpu_timer;                            //! Timer for ADC callback (Default 100ms / 10Hz)

 static void init_callbacks(void) {
    #if HAS_PI_ADC || HAS_ADS1115
        add_repeating_timer_ms(-100, adc_sample_callback, NULL, &adc_timer); //10Hz - 100ms
    
    #endif
    
    #if HAS_HMC5883L
        add_repeating_timer_ms(-100, gy271_callback, NULL, &gy271_timer); //10Hz - 100ms
    #endif
    
    #if HAS_MPU6050 == true
        add_repeating_timer_ms(-50, mpu6050_callback, NULL, &mpu_timer); //20Hz - 50ms
    #endif
}

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

/** @} */ // end of callbacks

//Static bools to prevent resource contention of missing components
static bool ads_written = false;                                    //! Will skip trying to re-acquire and write to sensor mutex if the ADS1115 is disconnected
static bool hmc5883l_written = false;                               //! Will skip trying to re-acquire and write to sensor mutex if the HMC5883L is disconnected
static bool mpu6050_written = false;                                //! Will skip trying to re-acquire and write to sensor mutex if the MPU6050 is disconnected
static bool pico_adc_written = false;                               //! Will skip trying to re-acquire and write to sensor mutex if the Pi Pico ADC is unused


/*!
 * @brief Convert raw ADS1115 value to voltage
 * @param raw 16-bit raw ADC value
 * @return Voltage in volts
 */
static inline float ads_voltage(uint16_t raw) {
    return (int16_t)raw * (4.096f / 32768.0); //±4.096V range
}

/*!
 * @brief Reads all ADC's and stores the resultant data in the sensor_readings struct.
 * @post sensor_readings contains updated ADC data.
 */
static void read_adc_data(void) {
    static uint8_t current_channel = 0;

    if (HAS_ADS1115) {
        uint16_t raw = read_ads_channel(current_channel);
        float voltage = ads_voltage(raw);
        
        mutex_enter_blocking(&sensor_readings.data_mutex);
        sensor_readings.adc_values[current_channel] = voltage;
        mutex_exit(&sensor_readings.data_mutex);

        current_channel = (current_channel + 1) % 4;
        
        // Start next conversion
        uint16_t config = ADS1115_BASE_CONFIG | (0x4000 | (current_channel << 12));
        uint8_t config_bytes[2] = {config >> 8, config & 0xFF};
        i2c_write_blocking(I2C_PORT, ADS1115_ADDR, config_bytes, 2, true);
    }

    // Read Pico's ADC2 (existing code)
    if (HAS_PI_ADC) {
        adc_select_input(2);
        float adc2_val = adc_read() * 3.3f / 4096.0f;
        mutex_enter_blocking(&sensor_readings.data_mutex);
        sensor_readings.adc_values[4] = adc2_val;
        mutex_exit(&sensor_readings.data_mutex);
    }
}

/*!
 * @brief Read specified channel from ADS1115 ADC.
 * @param channel ADC channel to read (0-3).
 * @return Raw 16-bit ADC value.
 */
static inline uint16_t read_ads_channel(uint8_t channel) {
    uint16_t config = ADS1115_BASE_CONFIG | (0x4000 | (channel << 12)); //Set MUX
    uint8_t config_bytes[2] = {config >> 8, config & 0xFF};
    i2c_write_blocking(I2C_PORT, ADS1115_ADDR, config_bytes, 2, true);
    sleep_ms(8); //Wait for conversion
    uint8_t reg = 0x00; //Conversion register
    i2c_write_blocking(I2C_PORT, ADS1115_ADDR, &reg, 1, true);
    uint8_t buffer[2];
    i2c_read_blocking(I2C_PORT, ADS1115_ADDR, buffer, 2, false);
    return (buffer[0] << 8) | buffer[1];
}

/*!
 * @brief Function that reads the HMC5883L data.
 * @details Uses I2C bus to get magenetic field measurements.
 * @post The sensor_readings structure contains the latest magnetic field.
 */
static void read_hmc5883l_data(void) {
    if (HAS_HMC5883L) {
        uint8_t buffer[6];
    
        if(mutex_try_enter(&i2c_mutex, NULL)) {
            i2c_write_blocking_until(I2C_PORT, HMC5883L_ADDR, (uint8_t[]){HMC5883L_DATA}, 1, true, time_us_64() + 1000);
            i2c_read_blocking_until(I2C_PORT, HMC5883L_ADDR, buffer, 6, false, time_us_64() + 2000);
            mutex_exit(&i2c_mutex);
            mutex_enter_blocking(&sensor_readings.data_mutex);
            sensor_readings.mag[0] = (buffer[0] << 8) | buffer[1];
            sensor_readings.mag[1] = (buffer[4] << 8) | buffer[5]; //HMC5883L XYZ order
            sensor_readings.mag[2] = (buffer[2] << 8) | buffer[3];
            mutex_exit(&sensor_readings.data_mutex);
        }
    }

    else {
        //Only write the invalid values once
        if (!hmc5883l_written) {
            mutex_enter_blocking(&sensor_readings.data_mutex);
            sensor_readings.mag[0] = ~0;
            sensor_readings.mag[1] = ~0;
            sensor_readings.mag[2] = ~0;
            mutex_exit(&sensor_readings.data_mutex);

            hmc5883l_written = true;
        }
    }

}

/*!
 * @brief Function that reads the MPU6050 data.
 * @details Uses the I2C bus to get linear acceleration and angular velocity measurements.
 * @post The sensor_readings structure contains the latest acceleration and rpy values.
 */
static void read_mpu6050_data(void) {
    if (HAS_MPU6050) {
        uint8_t buffer[14];
    
        if(mutex_try_enter(&i2c_mutex, NULL)) {
            i2c_write_blocking_until(I2C_PORT, MPU6050_ADDR, 
                               (uint8_t[]){MPU6050_ACCEL_XOUT_H}, 1, false, time_us_64() + 1000);
            i2c_read_blocking_until(I2C_PORT, MPU6050_ADDR, buffer, 14, false, time_us_64() + 2000);
            mutex_exit(&i2c_mutex);
            mutex_enter_blocking(&sensor_readings.data_mutex);
            sensor_readings.accel[0] = (buffer[0] << 8) | buffer[1];
            sensor_readings.accel[1] = (buffer[2] << 8) | buffer[3];
            sensor_readings.accel[2] = (buffer[4] << 8) | buffer[5];
            sensor_readings.gyro[0] = (buffer[8] << 8) | buffer[9];
            sensor_readings.gyro[1] = (buffer[10] << 8) | buffer[11];
            sensor_readings.gyro[2] = (buffer[12] << 8) | buffer[13];
            mutex_exit(&sensor_readings.data_mutex);
        }
    }

    else {
        if (!mpu6050_written) {
            mutex_enter_blocking(&sensor_readings.data_mutex);
            sensor_readings.accel[0] = ~0;
            sensor_readings.accel[1] = ~0;
            sensor_readings.accel[2] = ~0;
            sensor_readings.gyro[0] = ~0;
            sensor_readings.gyro[1] = ~0;
            sensor_readings.gyro[2] = ~0;
            mutex_exit(&sensor_readings.data_mutex);

            mpu6050_written = true;
        }
    }
}