/*!
 * \file Robohand_i2c.c
 * \brief Provides I2C integration and utility functions.
 * \details Provides I2C, DMA, and sensor-specific functionality.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#include "Robohand.h"
#include "Robohand_i2c.h"

#include "hardware/i2c.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"

#include <stdio.h>
#include <stdatomic.h>

/**
 * @brief Atomic flag register for coordinating I2C access
 * @details May change at any point due to interrupt handlers
 */
_Atomic uint8_t i2c_operation_flags = 0;

/**
 * @brief Mutex to protect I2C bus access from concurrent operations
 */
mutex_t i2c_mutex;
static uint32_t i2c_mutex_owner = 0;
static uint32_t i2c_mutex_acquire_time = 0;

bool safe_mutex_enter(mutex_t *mutex, uint32_t timeout_ms, const char *function_name) {
    bool success;
    if (timeout_ms == 0) {
        mutex_enter_blocking(mutex);
        success = true;
    } else {
        success = mutex_enter_timeout_ms(mutex, timeout_ms);
    }
    
    if (success) {
        i2c_mutex_owner = (uint32_t)function_name;
        i2c_mutex_acquire_time = time_us_32();
        if (DEBUG > 0) {
            printf("I2C mutex acquired by %s at %lu\r\n", function_name, i2c_mutex_acquire_time);
        }
    } else if (DEBUG > 0) {
        printf("Failed to acquire I2C mutex in %s (currently owned by %s for %lu us)\r\n", 
               function_name, (const char*)i2c_mutex_owner, 
               time_us_32() - i2c_mutex_acquire_time);
    }
    
    return success;
}

// Replace mutex_exit for i2c_mutex with this
void safe_mutex_exit(mutex_t *mutex, const char *function_name) {
    if (DEBUG > 0) {
        printf("I2C mutex released by %s after %lu us\r\n", 
               function_name, time_us_32() - i2c_mutex_acquire_time);
    }
    i2c_mutex_owner = 0;
    mutex_exit(mutex);
}

/**
 * @brief Initializes the I2C interface with appropriate configuration
 */
void robohand_init_i2c(void) {
    if (DEBUG > 0) {
        printf("Initializing I2C.\r\n");
    }

    i2c_init(I2C_PORT, 400000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    i2c_scan_bus();
    mutex_init(&i2c_mutex);

    if (DEBUG > 0) {
        printf("I2C initialized.\r\n");
    }
}

/**
 * @brief Write to I2C device with retry mechanism
 * 
 * @param addr I2C device address
 * @param src Source data buffer
 * @param len Length of data to write
 * @param retain_bus Whether to retain control of the bus after transaction
 * @param timeout Timeout in microseconds
 * @return true if write was successful, false otherwise
 */
bool i2c_read_with_retry(uint8_t addr, uint8_t* dest, size_t len, bool retain_bus, uint64_t timeout) {
    if (!HAS_I2C || !dest) return false;
    
    const int MAX_RETRIES = 3;
    int retries = MAX_RETRIES;
    
    while (retries > 0) {
        int result = i2c_read_blocking_until(I2C_PORT, addr, dest, len, retain_bus, time_us_64() + timeout);
        
        if (result == len) {
            return true;  // Success
        }
        
        // Log specific error types
        if (DEBUG > 0) {
            if (result == PICO_ERROR_TIMEOUT) {
                printf("I2C[0x%02X]: Read timeout (%d retries left)\r\n", addr, retries-1);
            } else if (result == PICO_ERROR_GENERIC) {
                printf("I2C[0x%02X]: Read generic error (%d retries left)\r\n", addr, retries-1);
            } else if (result >= 0 && result < len) {
                printf("I2C[0x%02X]: Incomplete read %d/%d bytes (%d retries left)\r\n", 
                    addr, result, len, retries-1);
            } else {
                printf("I2C[0x%02X]: Read unknown error %d (%d retries left)\r\n", 
                       addr, result, retries-1);
            }
        }
        
        // Add exponential backoff for retries
        sleep_ms(5 * (1 << (MAX_RETRIES - retries)));
        retries--;
    }
    
    // All retries failed
    if (DEBUG > 0) {
        printf("I2C[0x%02X]: Read failed after %d attempts\r\n", addr, MAX_RETRIES);
    }
    
    return false;
}

/**
 * @brief Read from a specific register on an I2C device
 * 
 * @param dev_addr I2C device address
 * @param reg_addr Register address to read from
 * @param data Buffer to store read data
 * @param len Length of data to read
 * @return true if read was successful, false otherwise
 */
bool i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
    if (!safe_mutex_enter(&i2c_mutex, 100, "i2c_read_reg")) {
        if (DEBUG > 0) {
            printf("i2c_read_reg: Failed to acquire mutex\r\n");
        }
        return false;
    }
    
    bool write_success = i2c_write_with_retry(dev_addr, &reg_addr, 1, true, 5000);
    if (!write_success && DEBUG > 0) {
        printf("i2c_read_reg: Failed to write register address 0x%02X to device 0x%02X\r\n", 
               reg_addr, dev_addr);
    }
    
    bool read_success = false;
    if (write_success) {
        read_success = i2c_read_with_retry(dev_addr, data, len, false, 5000);
        if (!read_success && DEBUG > 0) {
            printf("i2c_read_reg: Failed to read %d bytes from device 0x%02X register 0x%02X\r\n", 
                   len, dev_addr, reg_addr);
        }
    }
    
    safe_mutex_exit(&i2c_mutex, "i2c_read_reg");
    return write_success && read_success;
}

/*!
 * @brief Write to the I2C port with retry mechanism.
 * @param addr I2C device address.
 * @param src Source buffer containing data to write.
 * @param len Number of bytes to write.
 * @param retain_bus Whether to retain the bus for subsequent operations.
 * @param timeout Operation timeout in microseconds.
 * @return True if write succeeded, false otherwise.
 */
bool i2c_write_with_retry(uint8_t addr, const uint8_t* src, size_t len, bool retain_bus, uint64_t timeout) {
    if (!HAS_I2C || !src) return false;
    
    const int MAX_RETRIES = 3;
    int retries = MAX_RETRIES;
    
    while (retries > 0) {
        int result = i2c_write_blocking_until(I2C_PORT, addr, src, len, retain_bus, time_us_64() + timeout);
        
        if (result == len) {
            return true;  // Success
        }
        
        // Log specific error types with consistent format
        if (DEBUG > 0) {
            if (result == PICO_ERROR_TIMEOUT) {
                printf("I2C[0x%02X]: Timeout error (%d retries left)\r\n", addr, retries-1);
            } else if (result == PICO_ERROR_GENERIC) {
                printf("I2C[0x%02X]: Generic error (%d retries left)\r\n", addr, retries-1);
            } else if (result >= 0 && result < len) {
                printf("I2C[0x%02X]: Incomplete write %d/%d bytes (%d retries left)\r\n", 
                       addr, result, len, retries-1);
            } else {
                printf("I2C[0x%02X]: Unknown error %d (%d retries left)\r\n", 
                       addr, result, retries-1);
            }
        }
        
        // Add exponential backoff for retries
        sleep_ms(5 * (1 << (MAX_RETRIES - retries)));
        retries--;
    }
    
    // All retries failed
    if (DEBUG > 0) {
        printf("I2C[0x%02X]: Write failed after %d attempts\r\n", addr, MAX_RETRIES);
    }
    
    return false;
}

/**
 * @brief Write to a specific register on an I2C device
 * 
 * @param dev_addr I2C device address
 * @param reg_addr Register address to write to
 * @param value Value to write
 * @return true if write was successful, false otherwise
 */
bool i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
    uint8_t data[2] = {reg_addr, value};
    
    if (!safe_mutex_enter(&i2c_mutex, 100, "i2c_write_reg")) {
        return false;
    }
    
    bool success = i2c_write_with_retry(dev_addr, data, 2, false, 5000);
    
    safe_mutex_exit(&i2c_mutex, "i2c_write_reg");
    return success;
}

/**
 * @brief Scan I2C bus for connected devices
 */
void i2c_scan_bus(void) {
    printf("\r\nScanning I2C bus:\r\n");
    bool found_any = false;
    
    for (uint8_t addr = 0; addr < 128; addr++) {
        uint8_t dummy = 0;
        int ret = i2c_write_blocking(I2C_PORT, addr, &dummy, 1, false);
        if (ret >= 0) {
            found_any = true;
            printf("  Device found at address 0x%02X", addr);
            
            // Identify known devices
            if (addr == BME280_ADDR) printf(" (BME280)");
            if (addr == MPU6050_ADDR) printf(" (MPU6050)");
            if (addr == QMC5883L_ADDR) printf(" (QMC5883L)");
            if (addr == ADS1115_ADDR) printf(" (ADS1115)");
            
            printf("\r\n");
        }
    }
    
    if (!found_any) {
        printf("  No I2C devices found! Check wiring.\r\n");
    }
    printf("\r\n");
}

/**
 * @brief Check if all configured I2C devices are responding
 * 
 * @return true if all devices are responsive, false otherwise
 */
bool i2c_check_devices(void) {   
    bool all_ok = true;

    if (!HAS_I2C) {
        return true;
    }
    
    if (safe_mutex_enter(&i2c_mutex, 50, "i2c_check_devices")) {

        all_ok = all_ok && i2c_check_ads1115();
        all_ok = all_ok && i2c_check_bme280();
        all_ok = all_ok && i2c_check_mpu6050();
        all_ok = all_ok && i2c_check_qmc5883l();
        
        safe_mutex_exit(&i2c_mutex, "i2c_check_devices");
    }
    
    else {
        if (DEBUG > 0) {
            printf("Failed to acquire I2C mutex for device check\r\n");
        }
        all_ok = false;
    }
    
    return all_ok;
}

/**
 * @brief Check if ADS1115 is responding
 * 
 * @return true if device is responsive, false otherwise
 */
bool i2c_check_ads1115(void) {
    bool all_ok = true;
    uint8_t dummy;

    if (HAS_ADS1115 && i2c_read_blocking_until(I2C_PORT, ADS1115_ADDR, &dummy, 1, false, time_us_64() + 1000) != 1) {
        if (DEBUG > 0) {
            printf("ADS1115 not responding\r\n");
        }

        all_ok = false;
    }

    return all_ok;
}

/**
 * @brief Check if BME280 is responding
 * 
 * @return true if device is responsive, false otherwise
 */
bool i2c_check_bme280(void) {
    bool all_ok = true;
    uint8_t dummy;

    if (HAS_BME280 && i2c_read_blocking_until(I2C_PORT, BME280_ADDR, &dummy, 1, false, time_us_64() + 1000) != 1) {
        if (DEBUG > 0) {
            printf("BME280 not responding\r\n");
        }

        all_ok = false;
    }

    return all_ok;
}

/**
 * @brief Check if MPU6050 is responding
 * 
 * @return true if device is responsive, false otherwise
 */
bool i2c_check_mpu6050(void) {
    bool all_ok = true;
    uint8_t dummy;

    if (HAS_MPU6050 && i2c_read_blocking_until(I2C_PORT, MPU6050_ADDR, &dummy, 1, false, time_us_64() + 1000) != 1) {
        if (DEBUG > 0) {
            printf("MPU6050 not responding\r\n");
        }

        all_ok = false;
    }

    return all_ok;
}

/**
 * @brief Check if QMC5883L is responding
 * 
 * @return true if device is responsive, false otherwise
 */
bool i2c_check_qmc5883l(void) {
    bool all_ok = true;
    uint8_t dummy;

    if (HAS_QMC5883L && i2c_read_blocking_until(I2C_PORT, QMC5883L_ADDR, &dummy, 1, false, time_us_64() + 1000) != 1) {
        if (DEBUG > 0) {
            printf("QMC5883L not responding\r\n");
        }

        all_ok = false;
    }

    return all_ok;
}

/**
 * @brief Attempt to recover I2C bus in case of issues
 */
void i2c_recover_bus(void) {
    if (!HAS_I2C) return;
    
    if (DEBUG > 0) {
        printf("Attempting I2C bus recovery\r\n");
    }
    
    // First, reinitialize the I2C hardware
    i2c_deinit(I2C_PORT);
    sleep_ms(50);
    i2c_init(I2C_PORT, 400000);
    
    // Toggle SCL manually to unstick any devices
    gpio_set_function(SCL_PIN, GPIO_FUNC_SIO);
    gpio_set_function(SDA_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(SCL_PIN, GPIO_OUT);
    gpio_set_dir(SDA_PIN, GPIO_OUT);
    
    // Send 9 clock pulses to reset any stuck devices
    for (int i = 0; i < 9; i++) {
        gpio_put(SCL_PIN, 0);
        sleep_us(5);
        gpio_put(SCL_PIN, 1);
        sleep_us(5);
    }
    
    // Send STOP condition
    gpio_put(SDA_PIN, 0);
    sleep_us(5);
    gpio_put(SCL_PIN, 1);
    sleep_us(5);
    gpio_put(SDA_PIN, 1);
    sleep_us(5);
    
    // Restore I2C function
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    
    if (DEBUG > 0) {
        printf("I2C bus recovery completed\r\n");
    }
}