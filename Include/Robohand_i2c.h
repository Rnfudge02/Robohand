/*!
 * \file Robohand_i2c.h
 * \brief Provides I2C integration and utility functions.
 * \details Provides I2C, DMA, and sensor-specific functionality.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#ifndef ROBOHAND_I2C_H
#define ROBOHAND_I2C_H

//Ensure C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

#include <stdatomic.h>
#include <stdint.h>

#include "hardware/i2c.h"
#include "pico/mutex.h"

/** @defgroup i2c_flags I2C Operation Flags
 *  @brief Flags used to signal required I2C operations.
 *  @{
 */
#define ADC_READ_FLAG (1 << 0)                                  ///< Bit signalling an ADC read is needed over the I2C bus
#define BME_READ_FLAG (1 << 1)                                  ///< Bit signalling a BME read is needed
#define MPU_READ_FLAG (1 << 2)                                  ///< Bit signalling a MPU read is needed over the I2C bus
#define QMC_READ_FLAG (1 << 3)                                  ///< Bit signalling a QMC read is needed over the I2C bus
  
/** @} */ // end of i2c_flags

/** @defgroup i2c_const I2C Constants
 *  @brief Constants used in driver code.
 *  @{
 */

#define HAS_ADS1115 false                                       ///< Whether the ADS1115 (ADC) is connected to the I2C bus
#define HAS_BME280 true                                         ///< Whether the BME280 altitude sensor is connected to the I2C bus
#define HAS_QMC5883L true                                       ///< Whether the QMC5883L (Magnometer) is connected to the i2c bus
#define HAS_MPU6050 true                                        ///< Whether the MPU6050 (Accelerometer) is connected to the i2c bus
#define HAS_I2C true                                            ///< Whether or not I2C initialization is required

#define I2C_PORT i2c0                                           ///< I2C port used for device connections(i2c0 on pico)
#define SDA_PIN 0                                               ///< GPIO pin for I2C SDA
#define SCL_PIN 1                                               ///< GPIO pin for I2C SCL

//I2C Addresses
#define ADS1115_ADDR 0x48                                       ///< I2C address of ADS1115 ADC
#define BME280_ADDR 0x76                                        ///< I2C address of BME280 pressure sensor
#define MPU6050_ADDR 0x68                                       ///< I2C address of MPU6050 IMU
#define QMC5883L_ADDR 0x0D                                      ///< I2C address of QMC5883L magnetometer

//QMC5883L Registers
#define QMC5883L_CONFIG_A 0x00                                  ///< Configuration register A
#define QMC5883L_CONFIG_B 0x01                                  ///< Configuration register B
#define QMC5883L_MODE 0x02                                      ///< Mode register
#define QMC5883L_DATA 0x03                                      ///< Data output register

#define QMC5883L_MODE_CONTINUOUS 0x00                           ///< Continious sampling mode

//ADS1115 Configuration Macros
#define ADS1115_OS_SINGLE   0x8000                              ///< Start single-conversion
#define ADS1115_MUX_AIN0    0x4000                              ///< AIN0 vs GND
#define ADS1115_MUX_AIN1    0x5000                              ///< AIN1 vs GND
#define ADS1115_MUX_AIN2    0x6000                              ///< AIN2 vs GND
#define ADS1115_MUX_AIN3    0x7000                              ///< AIN3 vs GND
#define ADS1115_FSR_4V096   0x0200                              ///< ±4.096V range
#define ADS1115_MODE_SINGLE 0x0100                              ///< Single-shot mode
#define ADS1115_DR_128SPS   0x0080                              ///< 128 samples/sec
#define ADS1115_COMP_MODE   0x0000                              ///< Traditional comparator
#define ADS1115_COMP_POL    0x0000                              ///< Active low
#define ADS1115_COMP_LAT    0x0000                              ///< Non-latching
#define ADS1115_COMP_QUE    0x0003                              ///< Disable comparator

#define ADS1115_BASE_CONFIG (ADS1115_OS_SINGLE | ADS1115_FSR_4V096 | ADS1115_MODE_SINGLE | \
    ADS1115_DR_128SPS | ADS1115_COMP_MODE | ADS1115_COMP_POL | \
    ADS1115_COMP_LAT | ADS1115_COMP_QUE)                        ///< ADS1115 base configuration

//MPU6050 Registers
#define MPU6050_ACCEL_XOUT_H 0x3B                               ///< Accelerometer data register

/** @} */ // end of i2c_const

extern _Atomic uint8_t i2c_operation_flags;
extern mutex_t i2c_mutex;

/**
 * @defgroup i2c_utils I2C Utility Functions
 * @brief Helper functions for I2C communication
 * @{
 */
void robohand_init_i2c(void);
bool i2c_write_with_retry(uint8_t addr, const uint8_t* src, size_t len, bool retain_bus, uint64_t timeout);
bool i2c_read_with_retry(uint8_t addr, uint8_t* dest, size_t len, bool retain_bus, uint64_t timeout);
bool i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len);
bool i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);
void i2c_scan_bus(void);
bool i2c_check_devices(void);
bool i2c_check_ads1115(void);
bool i2c_check_bme280(void);
bool i2c_check_mpu6050(void);
bool i2c_check_qmc5883l(void);
void i2c_recover_bus(void);
/** @} */

#ifdef __cplusplus
}
#endif

#endif // ROBOHAND_I2C_H