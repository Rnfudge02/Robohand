#include "Robohand_advanced.h"

#include <stdio.h>

#include <stdatomic.h>

#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"


typedef int channel_id;

static uint8_t mpu_buffer[14]; // Accel + gyro data
static uint16_t adc_buffer[DMA_ADC_SAMPLES];
static int pwm_dma_chan;  // Track PWM DMA channel
static uint16_t adc_buffer[DMA_ADC_SAMPLES];
static int adc_dma_chan;

//DMA channels for I2C0/I2C1
static int i2c_tx_dma_chan;
static int i2c_rx_dma_chan;

static dma_control dma;

bme280_calib_data bme280_calib;
static volatile uint8_t i2c_operation_flags = 0;                    //! Register coordinating I2C accesses, may change at any point
sensor_data sensor_readings;
static mutex_t i2c_mutex;

//Hardware initialization functions
static void robohand_init_i2c(void);
static void robohand_init_ads1115(void);
static void robohand_init_bme280(void);
static void robohand_init_mpu6050(void);
static void robohand_init_qmc5883l(void);

//DMA handler functions
void init_dma();

//Interrupt handler functions
static void init_interrupts();
static void ads1115_drdy_handler(uint gpio, uint32_t events);
static void bme280_drdy_handler(uint gpio, uint32_t events);
static void gy271_drdy_handler(uint gpio, uint32_t events);
static void mpu6050_drdy_handler(uint gpio, uint32_t events);

//Callback functions
static void init_callbacks();
static bool adc_sample_callback(const struct repeating_timer* t);
static bool bme280_callback(const struct repeating_timer* t);
static bool gy271_callback(const struct repeating_timer* t);
static bool mpu6050_callback(const struct repeating_timer* t);

//Reader functions
static inline uint16_t read_ads_channel(uint8_t channel);
static inline float ads_voltage(uint16_t raw);

static void read_adc_data(void);
static void read_bme280_calibration(void);
static void compensate_bme280_data(const uint8_t *raw_data, float *temperature, float *pressure, float *humidity);
static void read_bme280_raw_data(uint8_t* data);
static void read_mpu6050_data(void);
static void read_qmc5883l_data(void);

//Misc I2C function
static bool i2c_write_with_retry(uint8_t addr, const uint8_t* src, size_t len, bool retain_bus, uint64_t timeout);


void scan_i2c_bus();



static void robohand_init_mpu6050(void) {
    if (HAS_MPU6050) {
        if (DEBUG > 0) {
            printf("Configuring MPU6050.\r\n");
        }

        uint8_t wake[] = {0x6B, 0x00};
        i2c_write_with_retry(MPU6050_ADDR, wake, 2, false, 100000);
        uint8_t gyro_config[] = {0x1B, 0x08};
        i2c_write_with_retry(MPU6050_ADDR, gyro_config, 2, false, 100000);
        uint8_t accel_config[] = {0x1C, 0x10};
        i2c_write_with_retry(MPU6050_ADDR, accel_config, 2, false, 100000);

        if (DEBUG > 0) {
            printf("MPU6050 configuration finished.\r\n");
        }
    }
}

static void robohand_init_ads1115(void) {
    if (HAS_ADS1115) {
        if (DEBUG > 0) {
            printf("Configuring ADS1115.\r\n");
        }
    
        // Set default configuration (single-shot mode on AIN0)
        uint16_t config = ADS1115_BASE_CONFIG | ADS1115_MUX_AIN0;
        uint8_t config_bytes[2] = {config >> 8, config & 0xFF};
        i2c_write_with_retry(ADS1115_ADDR, config_bytes, 2, false, 100000);
    
        // Configure ALERT pin as input
        gpio_init(ADS1115_INT_PIN);
        gpio_set_dir(ADS1115_INT_PIN, GPIO_IN);
        gpio_pull_up(ADS1115_INT_PIN);
    
        if (DEBUG > 0) {
            printf("ADS1115 configuration finished.\r\n");
        }
    }
}

static void robohand_init_qmc5883l(void) {
    if (HAS_QMC5883L) {
        if (DEBUG > 0) {
            printf("Configuring QMC5883L.\r\n");
        }
    
        uint8_t qmc_data[2];

        // Set the Set/Reset period (recommended by datasheet)
        qmc_data[0] = 0x0B;  // Register 0x0B: Set/Reset Period
        qmc_data[1] = 0x01;  // Value 0x01 as per datasheet recommendation
        i2c_write_with_retry(QMC5883L_ADDR, qmc_data, 2, false, 100000);

        // Configure for continuous mode, 50Hz ODR, 512 OSR, 2G range
        qmc_data[0] = 0x09;  // Register 0x09: Control Register 1
        qmc_data[1] = 0x0D;  // Mode=01 (continuous), ODR=01 (50Hz), OSR=00 (512), RNG=00 (2G)
        i2c_write_with_retry(QMC5883L_ADDR, qmc_data, 2, false, 100000);

        if (DEBUG > 0) {
            printf("QMC5883L configuration finished.\r\n");
        }
    }
}

static void robohand_init_bme280(void) {
    if (HAS_BME280) {
        if (DEBUG > 0) {
            printf("Configuring BME280.\r\n");
        }

        uint8_t bme_data[2];

        //Set humidity oversampling to x1
        bme_data[0] = 0xF2;  //Humidity control register
        bme_data[1] = 0x01;  //Oversampling x1
        i2c_write_with_retry(BME280_ADDR, bme_data, 2, false, 100000);

        //Set temperature and pressure oversampling to x1, normal mode
        bme_data[0] = 0xF4;  //Control measurement register
        bme_data[1] = 0x27;  //T x1 (001), P x1 (001), Normal mode (11)
        i2c_write_with_retry(BME280_ADDR, bme_data, 2, false, 100000);

        //Set standby time to 1000ms, filter off
        bme_data[0] = 0xF5;  //Config register
        bme_data[1] = 0xA0;  //Standby 1000ms (101), filter off (000)
        i2c_write_with_retry(BME280_ADDR, bme_data, 2, false, 100000);
        read_bme280_calibration();

        if (DEBUG > 0) {
            printf("BME280 configuration finished.\r\n");
        }
    }
}

static void robohand_init_i2c(void) {
    if (DEBUG > 0) {
        printf("Initializing I2C.\r\n");
    }

    i2c_init(I2C_PORT, 400000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    scan_i2c_bus();
    mutex_init(&i2c_mutex);

    if (DEBUG > 0) {
        printf("I2C initialized.\r\n");
    }
}

void robohand_init_components(void) {

    robohand_init_i2c();

    if (HAS_PI_ADC) {
        if (DEBUG > 0) {
            printf("Initializing Pico ADCs.\r\n");
        }

        adc_init();
        adc_gpio_init(ADC2_PIN);

        if (DEBUG > 0) {
            printf("Pico ADCs initialized.\r\n");
        }
    }

    robohand_init_i2c();
    robohand_init_ads1115();
    robohand_init_bme280();
    robohand_init_mpu6050();
    robohand_init_qmc5883l();

    if (USE_DMA) {
        if (DEBUG > 0) {
            printf("Intitializing DMA backend.\r\n");
        }

        init_dma();

        if (DEBUG > 0) {
            printf("DMA backend initialized.\r\n");
        }
    }

    if (USE_INTERRUPTS) {
        if (DEBUG > 0) {
            printf("Intitializing interrupt backend.\r\n");
        }

        init_interrupts();

        if (DEBUG > 0) {
            printf("Interrupt backend initialized.\r\n");
        }
    }

    if (USE_CALLBACKS) {
        if (DEBUG > 0) {
            printf("Intitializing callback backend.\r\n");
        }

        init_callbacks();

        if (DEBUG > 0) {
            printf("Callback backend initialized.\r\n");
        }
    }
}

void robohand_read() {
    //Handle sensor reads via flags
    if((i2c_operation_flags & ADC_READ_FLAG) && HAS_I2C) {
        read_adc_data();
        i2c_operation_flags &= ~ADC_READ_FLAG;
    }

    if ((i2c_operation_flags & BME_READ_FLAG) && HAS_I2C) {
        uint8_t raw_data[8];
        float temperature;
        float pressure;
        float humidity;
        read_bme280_raw_data(raw_data);
        compensate_bme280_data(raw_data, &temperature, &pressure, &humidity);
        i2c_operation_flags &= ~MPU_READ_FLAG;
    }

    if((i2c_operation_flags & MPU_READ_FLAG) && HAS_I2C) {
        read_mpu6050_data();
        i2c_operation_flags &= ~MPU_READ_FLAG;
    }
            
    if((i2c_operation_flags & QMC_READ_FLAG) && HAS_I2C) {
        read_qmc5883l_data();
        i2c_operation_flags &= ~QMC_READ_FLAG;
    }
}


// DMA initialization function
void init_dma() {
    // Claim DMA channels
    dma.i2c_tx.channel = (uint8_t) dma_claim_unused_channel(true);
    dma.i2c_rx.channel = (uint8_t) dma_claim_unused_channel(true);
    dma.adc.channel = (uint8_t) dma_claim_unused_channel(true);
    
    // Configure I2C TX DMA
    dma_channel_config c = dma_channel_get_default_config(dma.i2c_tx.channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, i2c_get_dreq(I2C_PORT, true));
    dma_channel_configure(dma.i2c_tx.channel, &c,
                          &i2c_get_hw(I2C_PORT)->data_cmd,
                          NULL, 0, false);

    // Configure I2C RX DMA
    c = dma_channel_get_default_config(dma.i2c_rx.channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, i2c_get_dreq(I2C_PORT, false));
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    dma_channel_configure(dma.i2c_rx.channel, &c,
                          NULL,
                          &i2c_get_hw(I2C_PORT)->data_cmd, 0, false);

    // Configure ADC DMA
    adc_fifo_setup(true, true, 1, false, false);
    c = dma_channel_get_default_config(dma.adc.channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_ADC);
    dma_channel_configure(dma.adc.channel, &c,
                          adc_buffer,
                          &adc_hw->fifo,
                          DMA_ADC_SAMPLES,
                          false);
}


uint16_t dma_read_ads_channel(uint8_t channel) {
    uint16_t config = (uint16_t)(ADS1115_BASE_CONFIG | (0x4000 | (channel << 12)));
    uint8_t config_bytes[2] = {config >> 8, config & 0xFF};
    
    //Write config using DMA (add trigger parameter)
    dma_channel_set_write_addr(dma.i2c_tx.channel, config_bytes, true);
    dma_channel_set_trans_count(dma.i2c_tx.channel, 2, true);
    dma_channel_wait_for_finish_blocking(dma.i2c_tx.channel);

    //Read result using DMA (add trigger parameter)
    uint8_t reg = 0x00;
    dma_channel_set_write_addr(dma.i2c_tx.channel, &reg, true);
    dma_channel_set_trans_count(dma.i2c_tx.channel, 1, true);
    dma_channel_wait_for_finish_blocking(dma.i2c_tx.channel);

    uint8_t result[2];
    dma_channel_set_read_addr(dma.i2c_rx.channel, result, true);
    dma_channel_set_trans_count(dma.i2c_rx.channel, 2, true);
    dma_channel_wait_for_finish_blocking(dma.i2c_rx.channel);

    return (uint16_t)((result[0] << 8) | result[1]);
}

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
    
    //Configure TX channel (I2C writes)
    dma_channel_config c = dma_channel_get_default_config(i2c_tx_dma_chan);
    channel_config_set_dreq(&c, I2C_DMA_TX);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    dma_channel_configure(i2c_tx_dma_chan, &c,
        &i2c_get_hw(I2C_PORT)->data_cmd, // Write to I2C data register
        NULL, // Dest addr set per-transfer
        0,    // Count set later
        false);

    //Configure RX channel (I2C reads)
    c = dma_channel_get_default_config(i2c_rx_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, I2C_DMA_RX);
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

void init_adc_dma() {
    adc_dma_chan = dma_claim_unused_channel(true);
    adc_fifo_setup(true, true, 1, false, false);
    
    dma_channel_config c = dma_channel_get_default_config(adc_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, I2C_DMA_TX);
    
    dma_channel_configure(adc_dma_chan, &c,
        adc_buffer,        // Dest
        &adc_hw->fifo,     // Src
        DMA_ADC_SAMPLES,       // Count
        true);             // Start

    irq_set_exclusive_handler(ADC_IRQ_FIFO, &adc_isr);
    irq_set_enabled(ADC_IRQ_FIFO, true);
}

/** @defgroup interrupts Interrupts
 *  @brief Used to eliminate polling. Change the CMake macro -DUSE_INTERRUPTS=ON to utilize this backend.
 *  @{
 */

static void init_interrupts(void) {
    if (HAS_ADS1115) {
        //Configure ALERT pin
        gpio_init(ADS1115_INT_PIN);
        gpio_set_dir(ADS1115_INT_PIN, GPIO_IN);
        gpio_pull_up(ADS1115_INT_PIN);
        gpio_set_irq_enabled_with_callback(ADS1115_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &ads1115_drdy_handler);

        //Start first conversion
        uint8_t channel = 0;
        uint16_t config = (uint16_t)(ADS1115_BASE_CONFIG | (0x4000 | (channel << 12)));
        uint8_t config_bytes[2] = {config >> 8, config & 0xFF};
        i2c_write_with_retry(ADS1115_ADDR, config_bytes, 2, true, 2000);
    }

    if (HAS_MPU6050) {
        if (DEBUG > 0) {
            printf("Configuring BME280.\r\n");
        }

        //Enable Data Ready Interrupt
        uint8_t int_enable[] = {0x38, 0x01}; // INT_ENABLE register
        i2c_write_with_retry(MPU6050_ADDR, int_enable, 2, false, 2000);

        //Configure GPIO interrupt
        gpio_init(BME280_INT_PIN);
        gpio_set_dir(BME280_INT_PIN, GPIO_IN);
        gpio_pull_up(BME280_INT_PIN);
        gpio_set_irq_enabled_with_callback(BME280_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &bme280_drdy_handler);
    }

    if (HAS_MPU6050) {
        if (DEBUG > 0) {
            printf("Configuring MPU6050.\r\n");
        }

        //Enable Data Ready Interrupt
        uint8_t int_enable[] = {0x38, 0x01}; // INT_ENABLE register
        i2c_write_with_retry(MPU6050_ADDR, int_enable, 2, false, 2000);

        //Configure GPIO interrupt
        gpio_init(MPU6050_INT_PIN);
        gpio_set_dir(MPU6050_INT_PIN, GPIO_IN);
        gpio_pull_up(MPU6050_INT_PIN);
        gpio_set_irq_enabled_with_callback(MPU6050_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &mpu6050_drdy_handler);
    }

    if (HAS_QMC5883L) {
        //Interrupt setup - Assuming pin will be held low when data ready, could this be confirmed?
        gpio_init(QMC5883L_INT_PIN);
        gpio_set_dir(QMC5883L_INT_PIN, GPIO_IN);
        gpio_pull_up(QMC5883L_INT_PIN);
        gpio_set_irq_enabled_with_callback(QMC5883L_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &gy271_drdy_handler);
    }
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
 * @brief BME280 data read handler (responds to physical interrupt).
 * @param gpio GPIO pin number.
 * @param events Triggering events.
 */
static void bme280_drdy_handler(uint gpio, uint32_t events) {
    if (HAS_BME280) {
        if(gpio == BME280_INT_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
            atomic_fetch_or_explicit(&i2c_operation_flags, QMC_READ_FLAG, memory_order_relaxed);
        }
    }

    else {
        return;
    }
}

/*!
 * @brief GY271 data read handler (responds to physical interrupt).
 * @param gpio GPIO pin number.
 * @param events Triggering events.
 */
static void gy271_drdy_handler(uint gpio, uint32_t events) {
    if (HAS_QMC5883L) {
        if(gpio == QMC5883L_INT_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
            atomic_fetch_or_explicit(&i2c_operation_flags, QMC_READ_FLAG, memory_order_relaxed);
        }
    }

    else {
        return;
    }
}

/*!
 * @brief MPU6050 data read handler (responds to physical interrupt).
 * @param gpio GPIO pin number.
 * @param events Triggering events.
 */
static void mpu6050_drdy_handler(uint gpio, uint32_t events) {
    if (HAS_MPU6050) {
        if (gpio == MPU6050_INT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
            atomic_fetch_or_explicit(&i2c_operation_flags, MPU_READ_FLAG, memory_order_relaxed);
        }
    }

    else {
        return;
    }
}

/** @} */ // end of interrupts

/** @defgroup callbacks Callbacks
 *  @brief Used as a fallback if no other implementations are available (USE_CALLBACKS macro is true).
 *  @{
 */

static struct repeating_timer adc_timer;                            //! Timer for ADC callback (Default 100ms / 10Hz)
static struct repeating_timer bme_timer;                            //! Timer for BME280 pressure sensor (Default 100ms / 10Hz)
static struct repeating_timer gy271_timer;                          //! Timer for QMC5883L callback (Default 50ms / 20Hz)
static struct repeating_timer mpu_timer;                            //! Timer for MPU6050 callback (Default 100ms / 10Hz)

/*!
 * @brief ADC data read callback.
 * @param t Timer structure.
 * @return true if the callback should continue running.
 */
static void init_callbacks(void) {
    if (HAS_PI_ADC || HAS_ADS1115) {
        add_repeating_timer_ms(-100, &adc_sample_callback, NULL, &adc_timer); //10Hz - 100ms
    }
    
    if (HAS_BME280) {
        add_repeating_timer_ms(-100, &bme280_callback, NULL, &bme_timer); //10Hz - 100ms
    }
    
    if (HAS_MPU6050) {
        add_repeating_timer_ms(-50, &mpu6050_callback, NULL, &mpu_timer); //20Hz - 50ms
    }

    if (HAS_QMC5883L) {
        add_repeating_timer_ms(-100, &gy271_callback, NULL, &gy271_timer); //10Hz - 100ms
    }
}

/*!
 * @brief ADC data read callback.
 * @param t Timer structure.
 * @return true if the callback should continue running.
 */
static bool adc_sample_callback(const struct repeating_timer* t) {
    (void)t;    //Cast to void to avoid static analysis errors

    if (HAS_ADC) {
        atomic_fetch_or_explicit(&i2c_operation_flags, ADC_READ_FLAG, memory_order_relaxed);
        return true;
    }

    else {
        return false;
    }
}

/*!
 * @brief BME280 data read callback.
 * @param t Timer structure.
 * @return true if the callback should continue running.
 */
static bool bme280_callback(const struct repeating_timer* t) {
    (void)t;    //Cast to void to avoid static analysis errors

    if (HAS_BME280) {
        atomic_fetch_or_explicit(&i2c_operation_flags, BME_READ_FLAG, memory_order_relaxed);
        return true;
    }

    else {
        return false;
    }
}

/*!
 * @brief GY271 data read callback.
 * @param t Timer structure.
 * @return true if the callback should continue running.
 */
static bool gy271_callback(const struct repeating_timer* t) {
    (void)t;    //Cast to void to avoid static analysis errors

    if (HAS_QMC5883L) {
        atomic_fetch_or_explicit(&i2c_operation_flags, QMC_READ_FLAG, memory_order_relaxed);
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
static bool mpu6050_callback(const struct repeating_timer* t) {
    (void)t;    //Cast to void to avoid static analysis errors

    if (HAS_MPU6050) {
        atomic_fetch_or_explicit(&i2c_operation_flags, MPU_READ_FLAG, memory_order_relaxed);
        return true;
    }

    else {
        return false;
    }
}

/** @} */ // end of callbacks

/*!
 * @brief Convert raw ADS1115 value to voltage
 * @param raw 16-bit raw ADC value
 * @return Voltage in volts
 */
static inline float ads_voltage(uint16_t raw) {
    return (int16_t)raw * (4.096f / 32768.0f); //±4.096V range
}

/*!
 * @brief Reads all ADC's and stores the resultant data in the sensor_readings struct.
 * @post sensor_readings contains updated ADC data.
 */
static void read_adc_data(void) {
    static uint8_t current_channel = 0;

    if (HAS_ADS1115) {
        uint16_t raw = ~0;
        if (USE_DMA) {
            raw = dma_read_ads_channel(current_channel);

        }
        
        else {
           raw = read_ads_channel(current_channel);
        }
        
        float voltage = ads_voltage(raw);
        
        mutex_enter_blocking(&sensor_readings.data_mutex);
        sensor_readings.adc_values[current_channel] = voltage;
        mutex_exit(&sensor_readings.data_mutex);

        current_channel = (current_channel + 1) % 4;
        
        //Start next conversion
        uint16_t config = (uint16_t)(ADS1115_BASE_CONFIG | (0x4000 | (current_channel << 12)));
        uint8_t config_bytes[2] = {config >> 8, config & 0xFF};
        i2c_write_with_retry(ADS1115_ADDR, config_bytes, 2, true, 2000);
    }

    //Read Pico's ADC2
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
    uint16_t config = (uint16_t) (ADS1115_BASE_CONFIG | (0x4000 | (channel << 12))); //Set MUX
    uint8_t config_bytes[2] = {config >> 8, config & 0xFF};
    i2c_write_with_retry(ADS1115_ADDR, config_bytes, 2, true, 4000);
    sleep_ms(8); //Wait for conversion
    uint8_t reg = 0x00; //Conversion register
    i2c_write_with_retry(ADS1115_ADDR, &reg, 1, true, 4000);
    uint8_t buffer[2];
    i2c_read_blocking(I2C_PORT, ADS1115_ADDR, buffer, 2, false);
    return (uint16_t)(buffer[0] << 8) | buffer[1];
}

/**
 * @brief Read calibration data from the BME280 sensor
 */
static void read_bme280_calibration(void) {
    uint8_t calib_buffer[32];  //Buffer for T/P (24 bytes) and H (7 bytes)

    //Read temperature and pressure calibration (0x88 to 0xA1)
    uint8_t reg = 0x88;
    i2c_write_with_retry(BME280_ADDR, &reg, 1, true, 100000);
    i2c_read_blocking(I2C_PORT, BME280_ADDR, calib_buffer, 24, false);

    //Read humidity calibration (0xE1 to 0xE7)
    reg = 0xE1;
    i2c_write_with_retry(BME280_ADDR, &reg, 1, true, 100000);
    i2c_read_blocking(I2C_PORT, BME280_ADDR, calib_buffer + 24, 7, false);

    //Parse temperature calibration
    bme280_calib.dig_T1 = (uint16_t)(calib_buffer[0] | (calib_buffer[1] << 8));
    bme280_calib.dig_T2 = (int16_t)(calib_buffer[2] | (calib_buffer[3] << 8));
    bme280_calib.dig_T3 = (int16_t)(calib_buffer[4] | (calib_buffer[5] << 8));

    //Parse pressure calibration
    bme280_calib.dig_P1 = (uint16_t)(calib_buffer[6] | (calib_buffer[7] << 8));
    bme280_calib.dig_P2 = (int16_t)(calib_buffer[8] | (calib_buffer[9] << 8));
    bme280_calib.dig_P3 = (int16_t)(calib_buffer[10] | (calib_buffer[11] << 8));
    bme280_calib.dig_P4 = (int16_t)(calib_buffer[12] | (calib_buffer[13] << 8));
    bme280_calib.dig_P5 = (int16_t)(calib_buffer[14] | (calib_buffer[15] << 8));
    bme280_calib.dig_P6 = (int16_t)(calib_buffer[16] | (calib_buffer[17] << 8));
    bme280_calib.dig_P7 = (int16_t)(calib_buffer[18] | (calib_buffer[19] << 8));
    bme280_calib.dig_P8 = (int16_t)(calib_buffer[20] | (calib_buffer[21] << 8));
    bme280_calib.dig_P9 = (int16_t)(calib_buffer[22] | (calib_buffer[23] << 8));

    // Parse humidity calibration
    bme280_calib.dig_H1 = calib_buffer[25];  // 0xE2
    bme280_calib.dig_H2 = (int16_t)(calib_buffer[24] | (calib_buffer[26] << 8));
    bme280_calib.dig_H3 = calib_buffer[27];
    bme280_calib.dig_H4 = (int16_t)((calib_buffer[28] << 4) | (calib_buffer[29] & 0x0F));
    bme280_calib.dig_H5 = (int16_t)((calib_buffer[30] << 4) | (calib_buffer[29] >> 4));
    bme280_calib.dig_H6 = (int8_t)calib_buffer[31];
}

/**
 * @brief Compensate BME280 raw data to get temperature, pressure, and humidity
 * @param raw_data Raw data from sensor (8 bytes)
 * @param temperature Pointer to store compensated temperature (in °C)
 * @param pressure Pointer to store compensated pressure (in Pa)
 * @param humidity Pointer to store compensated humidity (in %RH)
 * @note Based on default compensation code from datasheet
 */
static void compensate_bme280_data(const uint8_t *raw_data, float *temperature, float *pressure, float *humidity) {
    // Parse raw values (20-bit values shifted into 32-bit integers)
    int32_t raw_temp = (raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4);
    int32_t raw_press = (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4);
    int32_t raw_hum = (raw_data[6] << 8) | raw_data[7];

    // Temperature compensation
    int32_t var1 = (((raw_temp >> 3) - ((int32_t)bme280_calib.dig_T1 << 1)) * ((int32_t)bme280_calib.dig_T2)) >> 11;
    int32_t var2 = (((((raw_temp >> 4) - ((int32_t)bme280_calib.dig_T1)) * ((raw_temp >> 4) - ((int32_t)bme280_calib.dig_T1))) >> 12) * ((int32_t)bme280_calib.dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    *temperature = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

    // Pressure compensation
    int64_t p_var1 = ((int64_t)t_fine) - 128000;
    int64_t p_var2 = p_var1 * p_var1 * (int64_t)bme280_calib.dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)bme280_calib.dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)bme280_calib.dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)bme280_calib.dig_P3) >> 8) + ((p_var1 * (int64_t)bme280_calib.dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1) * ((int64_t)bme280_calib.dig_P1)) >> 33;
    if (p_var1 == 0) {
        *pressure = 0;
    }
    
    else {
        int64_t p = 1048576 - raw_press;
        p = (((p << 31) - p_var2) * 3125) / p_var1;
        p_var1 = (((int64_t)bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        p_var2 = (((int64_t)bme280_calib.dig_P8) * p) >> 19;
        *pressure = (float)(((p + p_var1 + p_var2) >> 8) + (((int64_t)bme280_calib.dig_P7) << 4)) / 256.0f;
    }

    // Humidity compensation (simplified)
    int32_t h = (t_fine - ((int32_t)76800));
    h = (((((raw_hum << 14) - (((int32_t)bme280_calib.dig_H4) << 20) - (((int32_t)bme280_calib.dig_H5) * h)) +
           ((int32_t)16384)) >> 15) * ((((((h * ((int32_t)bme280_calib.dig_H6)) >> 10) *
           (((h * ((int32_t)bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
           ((int32_t)bme280_calib.dig_H2) + 8192) >> 14);
    h = (h - (((((h >> 15) * (h >> 15)) >> 7) * ((int32_t)bme280_calib.dig_H1)) >> 4));
    h = (h < 0 ? 0 : h);
    h = (h > 419430400 ? 419430400 : h);
    *humidity = (float)(h >> 12) / 1024.0f;
}

/**
 * @brief Read raw data from the BME280 sensor
 * @param raw_data Pointer to array to store raw data (8 bytes: P, T, H)
 * @note raw_data[0-2]: Pressure (MSB, LSB, XLSB)
 * @note raw_data[3-5]: Temperature (MSB, LSB, XLSB)
 * @note raw_data[6-7]: Humidity (MSB, LSB)
 */
static void read_bme280_raw_data(uint8_t *raw_data) {
    uint8_t reg = 0xF7;  // Starting register for data output
    i2c_write_with_retry(BME280_ADDR, &reg, 1, true, 100000);
    i2c_read_blocking(I2C_PORT, BME280_ADDR, raw_data, 8, false);
}

/*!
 * @brief Function that reads the QMC5883L data.
 * @details Uses I2C bus to get magenetic field measurements.
 * @post The sensor_readings structure contains the latest magnetic field.
 */
static void read_qmc5883l_data(void) {
    if (HAS_QMC5883L) {
        uint8_t reg = 0x00;  // Starting register for data output
        uint8_t buffer[6];
    
        if(mutex_try_enter(&i2c_mutex, NULL)) {
            //Set the register pointer to 0x00
            i2c_write_with_retry(QMC5883L_ADDR, &reg, 1, true, 100000);
            i2c_read_blocking(I2C_PORT, QMC5883L_ADDR, buffer, 6, false); //Read 6 bytes (X_L, X_H, Z_L, Z_H, Y_L, Y_H)
            mutex_exit(&i2c_mutex);
            mutex_enter_blocking(&sensor_readings.data_mutex);
            sensor_readings.mag[0] = (int16_t)(buffer[0] | (buffer[1] << 8)); //Little endian packing for Cortex M0+
            sensor_readings.mag[1] = (int16_t)(buffer[2] | (buffer[3] << 8)); //QMC5883L XYZ order
            sensor_readings.mag[2] = (int16_t)(buffer[4] | (buffer[5] << 8));
            mutex_exit(&sensor_readings.data_mutex);
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
        if (mutex_try_enter(&i2c_mutex, NULL)) {
            bool wrote = i2c_write_with_retry(MPU6050_ADDR, (uint8_t[]){MPU6050_ACCEL_XOUT_H}, 1, false, 1000);
            int read = i2c_read_blocking_until(I2C_PORT, MPU6050_ADDR, buffer, 14, false, time_us_64() + 2000);
            mutex_exit(&i2c_mutex);

            if (wrote == 1 && read == 14) {
                mutex_enter_blocking(&sensor_readings.data_mutex);
                sensor_readings.accel[0] = (uint16_t)(buffer[0] << 8) | buffer[1];
                sensor_readings.accel[1] = (uint16_t)(buffer[2] << 8) | buffer[3];
                sensor_readings.accel[2] = (uint16_t)(buffer[4] << 8) | buffer[5];
                sensor_readings.gyro[0] = (uint16_t)(buffer[8] << 8) | buffer[9];
                sensor_readings.gyro[1] = (uint16_t)(buffer[10] << 8) | buffer[11];
                sensor_readings.gyro[2] = (uint16_t)(buffer[12] << 8) | buffer[13];
                mutex_exit(&sensor_readings.data_mutex);
            }
            
            else if (DEBUG > 0) {
                printf("MPU6050 read failed: wrote %d, read %d\r\n", wrote, read);
            }
        }
    }
}

/*!
 * @brief Write to the I2C port.
 * @details Retries write three times before quitting.
 * @return Whether write was successful or not.
 */
static bool i2c_write_with_retry(uint8_t addr, const uint8_t* src, size_t len, bool retain_bus, uint64_t timeout) {
    if (HAS_I2C) {
        int retries = 3;
        while(retries--) {
            int result = i2c_write_blocking_until(I2C_PORT, addr, src, len, retain_bus, time_us_64() + timeout);
            if(result == len) {
                return true;
            }

            if (DEBUG > 0 && result == len) {
                printf("I2C write to address %#04X completed successfully.\r\n", addr);
            }

            else if (DEBUG > 0 && result == PICO_ERROR_TIMEOUT) {
                printf("Timeout on I2C write error occured for address %#04X.\r\n", addr);
            }
    
            else if (DEBUG > 0 && result == PICO_ERROR_GENERIC) {
                printf("Generic Pico I2C write error occured for address %#04X.\r\n", addr);
            }
    
            else if (DEBUG > 0 && result != len) {
                printf("Incomplete write error occured for address %#04X.\r\n", addr);
            }
    
            if (DEBUG > 0 && retries != 0) {
                printf("attempts remaining: %d\r\n", retries);
            }

            sleep_ms(1);
        }

        return false;
    }
}

void scan_i2c_bus() {
    printf("Scanning I2C bus:\r\n");
    for (uint8_t addr = 0; addr < 128; addr++) {
        uint8_t dummy = 0;
        int ret = i2c_write_blocking(I2C_PORT, addr, &dummy, 1, false);
        if (ret >= 0) {
            printf("Device found at address 0x%02X\r\n", addr);
        }
    }
}