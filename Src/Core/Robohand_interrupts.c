/*!
 * \file Robohand_interrupts.c
 * \brief Interrupt backend, for balance of power with complexity.
 * \details Still a work-in-progress.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#include "Robohand.h"
#include "Robohand_interrupts.h"
#include "Robohand_i2c.h"

#include <stdatomic.h>
#include <stdio.h>

/**
 * @brief Initialize interrupt-based sensor data acquisition
 */
void init_interrupts(void) {
    // Initialize a reference to at least one callback handler
    // The Pico SDK requires a GPIO callback reference
    static bool callback_initialized = false;
    
    if (HAS_ADS1115) {
        // Configure ALERT/RDY pin
        gpio_init(ADS1115_INT_PIN);
        gpio_set_dir(ADS1115_INT_PIN, GPIO_IN);
        gpio_pull_up(ADS1115_INT_PIN);
        
        if (!callback_initialized) {
            // First pin sets up the callback
            gpio_set_irq_enabled_with_callback(ADS1115_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &ads1115_drdy_handler);
            callback_initialized = true;
        } else {
            // Additional pins just enable the interrupt
            gpio_set_irq_enabled(ADS1115_INT_PIN, GPIO_IRQ_EDGE_FALL, true);
        }

        // Start first conversion
        uint8_t channel = 0;
        uint16_t config = (uint16_t) (ADS1115_BASE_CONFIG | (0x4000 | (channel << 12)));
        uint8_t config_bytes[3] = {0x01, (uint8_t)(config >> 8), (uint8_t)(config & 0xFF)};
        i2c_write_with_retry(ADS1115_ADDR, config_bytes, 3, false, 2000);
    }

    if (HAS_BME280) {
        if (DEBUG > 0) {
            printf("Configuring BME280 interrupt.\r\n");
        }
        
        gpio_init(BME280_INT_PIN);
        gpio_set_dir(BME280_INT_PIN, GPIO_IN);
        gpio_pull_up(BME280_INT_PIN);
        
        if (!callback_initialized) {
            gpio_set_irq_enabled_with_callback(BME280_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &bme280_drdy_handler);
            callback_initialized = true;
        } else {
            gpio_set_irq_enabled(BME280_INT_PIN, GPIO_IRQ_EDGE_FALL, true);
        }
    }

    if (HAS_MPU6050) {
        // Configure MPU6050 to generate interrupts
        uint8_t int_config[] = {0x37, 0x20}; // INT_PIN_CFG: INT active low, clear on read
        i2c_write_with_retry(MPU6050_ADDR, int_config, 2, false, 2000);
        
        uint8_t int_enable[] = {0x38, 0x01}; // INT_ENABLE: Enable data ready interrupt
        i2c_write_with_retry(MPU6050_ADDR, int_enable, 2, false, 2000);

        // Configure GPIO - note active LOW for MPU6050
        gpio_init(MPU6050_INT_PIN);
        gpio_set_dir(MPU6050_INT_PIN, GPIO_IN);
        gpio_pull_up(MPU6050_INT_PIN); // Pull-up since interrupt is active LOW
        
        if (!callback_initialized) {
            gpio_set_irq_enabled_with_callback(MPU6050_INT_PIN, GPIO_IRQ_LEVEL_LOW, true, &mpu6050_drdy_handler);
            callback_initialized = true;
        } else {
            gpio_set_irq_enabled(MPU6050_INT_PIN, GPIO_IRQ_LEVEL_LOW, true);
        }
                                           
        if (DEBUG > 0) {
            printf("MPU6050 interrupt enabled on pin %d.\r\n", MPU6050_INT_PIN);
        }
    }

    if (HAS_QMC5883L) {
        // Interrupt setup - Assuming pin will be held low when data ready
        gpio_init(QMC5883L_INT_PIN);
        gpio_set_dir(QMC5883L_INT_PIN, GPIO_IN);
        gpio_pull_up(QMC5883L_INT_PIN);
        
        if (!callback_initialized) {
            gpio_set_irq_enabled_with_callback(QMC5883L_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &gy271_drdy_handler);
            callback_initialized = true;
        } else {
            gpio_set_irq_enabled(QMC5883L_INT_PIN, GPIO_IRQ_EDGE_FALL, true);
        }

        if (DEBUG > 0) {
            printf("QMC5883L interrupt enabled on pin %d.\r\n", QMC5883L_INT_PIN);
        }
    }
}

/**
 * @brief ADS1115 data ready interrupt handler
 * 
 * @param gpio GPIO pin number that triggered the interrupt
 * @param events Event type that triggered the interrupt
 */
static void ads1115_drdy_handler(uint gpio, uint32_t events) {
    if (gpio == ADS1115_INT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        uint8_t old_flags = atomic_load(&i2c_operation_flags);
        uint8_t new_flags = old_flags | ADC_READ_FLAG;
        atomic_store(&i2c_operation_flags, new_flags);
    }
}

/**
 * @brief BME280 data ready interrupt handler
 * 
 * @param gpio GPIO pin number that triggered the interrupt
 * @param events Event type that triggered the interrupt
 */
static void bme280_drdy_handler(uint gpio, uint32_t events) {
    if (HAS_BME280) {
        // Some BME280 interrupts are active high, event should match initialization
        if (gpio == BME280_INT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
            uint8_t old_flags = atomic_load(&i2c_operation_flags);
            uint8_t new_flags = old_flags | BME_READ_FLAG;
            atomic_store(&i2c_operation_flags, new_flags);
        }
    }
}

/**
 * @brief QMC5883L data ready interrupt handler
 * 
 * @param gpio GPIO pin number that triggered the interrupt
 * @param events Event type that triggered the interrupt
 */
static void gy271_drdy_handler(uint gpio, uint32_t events) {
    if (HAS_QMC5883L) {
        // QMC5883L interrupts are active low, so check for EDGE_FALL
        if (gpio == QMC5883L_INT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
            uint8_t old_flags = atomic_load(&i2c_operation_flags);
            uint8_t new_flags = old_flags | QMC_READ_FLAG;
            atomic_store(&i2c_operation_flags, new_flags);
        }
    }
}

/**
 * @brief MPU6050 data ready interrupt handler
 * 
 * @param gpio GPIO pin number that triggered the interrupt
 * @param events Event type that triggered the interrupt
 */
static void mpu6050_drdy_handler(uint gpio, uint32_t events) {
    if (HAS_MPU6050) {
        // MPU6050 INT pin is active LOW, so check for LEVEL_LOW
        if (gpio == MPU6050_INT_PIN && (events & GPIO_IRQ_LEVEL_LOW)) {
            uint8_t old_flags = atomic_load(&i2c_operation_flags);
            uint8_t new_flags = old_flags | MPU_READ_FLAG;
            atomic_store(&i2c_operation_flags, new_flags);
        }
    }
}