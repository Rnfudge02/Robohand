
#include "Robohand_i2c.h"
#include "Robohand_init.h"
#include "Robohand_dma.h"
#include "Robohand_callbacks.h"
#include "Robohand_common.h"
#include "Robohand_interrupts.h"
#include "Robohand_reader.h"

#include "pico/time.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#include <stdio.h>

/**
 * @defgroup initialization Hardware Initialization Functions
 * @brief Functions to initialize various hardware components
 * @{
 */

static bool robohand_init_ads1115(void);
static void robohand_init_bme280(void);
static bool robohand_init_mpu6050(void);
static bool robohand_init_qmc5883l(void);

/** @} */

/**
 * @brief Performs a periodic health check of the system
 * 
 * @details Checks I2C devices and attempts recovery if necessary
 */
void robohand_health_check(void) {
    static uint32_t last_check_time = 0;
    static uint32_t i2c_error_count = 0;
    static bool recovery_in_progress = false;
    
    uint32_t now = time_us_32();
    
    // Only check every 5 seconds to avoid excessive overhead
    if (now - last_check_time < 5000000 && !recovery_in_progress) {
        return;
    }
    
    last_check_time = now;
    
    // Check if I2C devices are responding
    if (HAS_I2C && !i2c_check_devices()) {
        i2c_error_count++;
        
        if (i2c_error_count >= 3 && !recovery_in_progress) {
            // Too many errors, attempt recovery
            recovery_in_progress = true;
            i2c_recover_bus();
            
            // Re-initialize the sensors
            if (HAS_ADS1115) robohand_init_ads1115();
            if (HAS_BME280) robohand_init_bme280();
            if (HAS_MPU6050) robohand_init_mpu6050();
            if (HAS_QMC5883L) robohand_init_qmc5883l();
            
            recovery_in_progress = false;
            i2c_error_count = 0;
            
            if (DEBUG > 0) {
                printf("I2C bus recovery completed\r\n");
            }
        }
    } else {
        // All good, reset error count
        i2c_error_count = 0;
    }
}

/**
 * @brief Initializes all hardware components based on configuration flags
 * 
 * @return true if initialization was successful, false otherwise
 */
bool robohand_init_components(void) {
    bool success = true;
    
    // Initialize Pico ADCs if enabled
    if (HAS_PI_ADC) {
        if (DEBUG > 0) {
            printf("Initializing Pico ADCs\r\n");
        }
        
        adc_init();
        adc_gpio_init(ADC2_PIN);
        
        if (DEBUG > 0) {
            printf("Pico ADCs initialized\r\n");
        }
    }
    
    // Initialize I2C
    if (HAS_I2C) {
        robohand_init_i2c();
    }
    
    // Initialize individual sensors
    if (HAS_ADS1115) {
        success &= robohand_init_ads1115();
    }
    
    if (HAS_BME280) {
        robohand_init_bme280();
    }
    
    if (HAS_MPU6050 && !robohand_init_mpu6050()) {
        if (DEBUG > 0) {
            printf("MPU6050 initialization failed, retrying...\r\n");
        }
        sleep_ms(100);

        if (!robohand_init_mpu6050()) {
            if (DEBUG > 0) {
                printf("MPU6050 initialization failed again\r\n");
            }
            success = false;
        }
    }
    
    if (HAS_QMC5883L && !robohand_init_qmc5883l()) {
        if (DEBUG > 0) {
            printf("QMC5883L initialization failed, retrying...\r\n");
        }
        sleep_ms(100);
        if (!robohand_init_qmc5883l()) {
            if (DEBUG > 0) {
                printf("QMC5883L initialization failed again\r\n");
            }
            success = false;
        }
    }
    
    // Initialize DMA if enabled
    if (USE_DMA) {
        if (DEBUG > 0) {
            printf("Initializing DMA backend\r\n");
        }
        
        init_dma();
        
        if (DEBUG > 0) {
            printf("DMA backend initialized\r\n");
        }
    }
    
    // Configure appropriate data acquisition method
    if (USE_INTERRUPTS) {
        if (DEBUG > 0) {
            printf("Initializing interrupt backend\r\n");
        }
        
        init_interrupts();
        
        if (DEBUG > 0) {
            printf("Interrupt backend initialized\r\n");
        }
    }
    else if (USE_CALLBACKS) {
        if (DEBUG > 0) {
            printf("Initializing callback backend\r\n");
        }
        
        init_callbacks();
        
        if (DEBUG > 0) {
            printf("Callback backend initialized\r\n");
        }
    }
    
    return success;
}

/**
 * @brief Initializes the MPU6050 accelerometer and gyroscope
 * 
 * @return true if initialization was successful, false otherwise
 */
static bool robohand_init_mpu6050(void) {
    if (!HAS_MPU6050) return false;
    
    if (DEBUG > 0) {
        printf("Configuring MPU6050...\r\n");
    }
    
    // Step 1: Reset device
    if (!i2c_write_reg(MPU6050_ADDR, 0x6B, 0x80)) { // PWR_MGMT_1, reset bit
        if (DEBUG > 0) {
            printf("Failed to reset MPU6050\r\n");
        }
        return false;
    }
    
    sleep_ms(100); // Wait for reset
    
    // Step 2: Wake up device and select best clock source
    if (!i2c_write_reg(MPU6050_ADDR, 0x6B, 0x01)) { // PWR_MGMT_1, clock source = PLL with X-axis gyro
        if (DEBUG > 0) {
            printf("Failed to wake up MPU6050\r\n");
        }
        return false;
    }
    
    sleep_ms(10);
    
    // Step 3: Disable sleep mode
    if (!i2c_write_reg(MPU6050_ADDR, 0x6C, 0x00)) { // PWR_MGMT_2, enable all axes
        if (DEBUG > 0) {
            printf("Failed to disable sleep mode on MPU6050\r\n");
        }
        return false;
    }
    
    // Step 4: Configure gyro for ±250 dps range
    if (!i2c_write_reg(MPU6050_ADDR, 0x1B, 0x00)) { // GYRO_CONFIG
        if (DEBUG > 0) {
            printf("Failed to configure MPU6050 gyro range\r\n");
        }
        return false;
    }
    
    // Step 5: Configure accel for ±2g range
    if (!i2c_write_reg(MPU6050_ADDR, 0x1C, 0x00)) { // ACCEL_CONFIG
        if (DEBUG > 0) {
            printf("Failed to configure MPU6050 accel range\r\n");
        }
        return false;
    }
    
    // Step 6: Configure digital low-pass filter
    if (!i2c_write_reg(MPU6050_ADDR, 0x1A, 0x03)) { // CONFIG, DLPF_CFG=3 (42Hz bandwidth)
        if (DEBUG > 0) {
            printf("Failed to configure MPU6050 DLPF\r\n");
        }
        return false;
    }
    
    // Step 7: Configure sample rate divider
    if (!i2c_write_reg(MPU6050_ADDR, 0x19, 0x04)) { // SMPLRT_DIV (1000 / (1 + 4) = 200Hz)
        if (DEBUG > 0) {
            printf("Failed to configure MPU6050 sample rate\r\n");
        }
        return false;
    }
    
    // Step 8: Configure interrupts
    if (!i2c_write_reg(MPU6050_ADDR, 0x37, 0x20)) { // INT_PIN_CFG: clear on read
        if (DEBUG > 0) {
            printf("Failed to configure MPU6050 interrupts\r\n");
        }
        return false;
    }
    
    if (!i2c_write_reg(MPU6050_ADDR, 0x38, 0x01)) { // INT_ENABLE: data ready
        if (DEBUG > 0) {
            printf("Failed to enable MPU6050 interrupts\r\n");
        }
        return false;
    }
    
    // Step 9: Verify WHO_AM_I register (should be 0x68)
    uint8_t who_am_i = 0;
    if (i2c_read_reg(MPU6050_ADDR, 0x75, &who_am_i, 1)) {
        if (who_am_i != 0x68) {
            if (DEBUG > 0) {
                printf("MPU6050 WHO_AM_I returned 0x%02X (expected 0x68)\r\n", who_am_i);
            }
            return false;
        }
    } else {
        if (DEBUG > 0) {
            printf("Failed to read MPU6050 WHO_AM_I register\r\n");
        }
        return false;
    }
    
    if (DEBUG > 0) {
        printf("MPU6050 configuration successful, WHO_AM_I = 0x%02X\r\n", who_am_i);
    }
    
    return true;
}

/**
 * @brief Initializes the ADS1115 analog-to-digital converter
 * 
 * @return true if initialization was successful, false otherwise
 */
static bool robohand_init_ads1115(void) {
    if (!HAS_ADS1115) return true;  // Not an error if not enabled
    
    if (DEBUG > 0) {
        printf("Configuring ADS1115\r\n");
    }
    
    // Try to detect ADS1115 by reading its ID register
    // Note: ADS1115 doesn't have an ID register, so we'll check if writing
    // to the configuration register works
    uint16_t config = ADS1115_BASE_CONFIG | ADS1115_MUX_AIN0;
    uint8_t config_bytes[3] = {0x01, config >> 8, config & 0xFF};  // Reg 0x01 + config
    
    if (!i2c_write_with_retry(ADS1115_ADDR, config_bytes, 3, false, 100000)) {
        if (DEBUG > 0) {
            printf("ADS1115 not detected or not responding\r\n");
        }
        return false;
    }
    
    // Configure ALERT pin as input
    gpio_init(ADS1115_INT_PIN);
    gpio_set_dir(ADS1115_INT_PIN, GPIO_IN);
    gpio_pull_up(ADS1115_INT_PIN);
    
    if (DEBUG > 0) {
        printf("ADS1115 configuration successful\r\n");
    }
    
    return true;
}

/**
 * @brief Initializes the QMC5883L magnetometer
 * 
 * @return true if initialization was successful, false otherwise
 */
static bool robohand_init_qmc5883l(void) {
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

/**
 * @brief Initializes the BME280 environmental sensor
 */
static void robohand_init_bme280(void) {
    if (!HAS_BME280) return;
    
    if (DEBUG > 0) {
        printf("Configuring BME/BMP280...\r\n");
    }

    // Reset the device first
    uint8_t reset_cmd[2] = {0xE0, 0xB6}; // Reset register and command
    i2c_write_with_retry(BME280_ADDR, reset_cmd, 2, false, 100000);
    sleep_ms(10); // Wait for reset to complete
    
    // Check chip ID (should be 0x60 for BME280 or 0x58 for BMP280)
    uint8_t id_reg = 0xD0;
    uint8_t chip_id;
    i2c_write_with_retry(BME280_ADDR, &id_reg, 1, true, 100000);
    i2c_read_blocking(I2C_PORT, BME280_ADDR, &chip_id, 1, false);
    
    bool is_bmp280 = false;
    
    if (chip_id == 0x60) {
        if (DEBUG > 0) {
            printf("BME280 detected (Chip ID: 0x%02X)\r\n", chip_id);
        }
    } else if (chip_id == 0x58) {
        if (DEBUG > 0) {
            printf("BMP280 detected (Chip ID: 0x%02X) - humidity not available\r\n", chip_id);
        }
        is_bmp280 = true;
    } else {
        if (DEBUG > 0) {
            printf("BME/BMP280 not found! Chip ID: 0x%02X\r\n", chip_id);
        }
        return;
    }
    
    // Set humidity oversampling to x4 (only relevant for BME280)
    if (!is_bmp280) {
        uint8_t hum_config[2] = {0xF2, 0x03}; // Humidity control register, x4 oversampling
        i2c_write_with_retry(BME280_ADDR, hum_config, 2, false, 100000);
    }
    
    // Set temperature and pressure oversampling to x4, normal mode
    uint8_t meas_config[2] = {0xF4, 0x6F}; // T x4 (110), P x4 (110), Normal mode (11)
    i2c_write_with_retry(BME280_ADDR, meas_config, 2, false, 100000);
    
    // Set standby time to 500ms, filter coefficient to 8
    uint8_t config_reg[2] = {0xF5, 0x54}; // Standby 500ms (100), filter x8 (101)
    i2c_write_with_retry(BME280_ADDR, config_reg, 2, false, 100000);

    // Read and store calibration data
    read_bme280_calibration();
    
    if (DEBUG > 0) {
        printf("BME/BMP280 configuration complete\r\n");
    }
}

