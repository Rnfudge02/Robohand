/*!
 * \file Robohand_reader.c
 * \brief Contains reader functions for sensor access.
 * \details Reader functions are called via the chosen backend.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#include "Robohand.h"
#include "Robohand_i2c.h"
#include "Robohand_reader.h"
#include "Robohand_struct.h"

#include "hardware/i2c.h"

#include <math.h>
#include <stdio.h>

/**
 * @brief Convert raw ADS1115 value to voltage
 * 
 * @param raw 16-bit raw ADC value
 * @return Voltage in volts
 */
float ads_voltage(uint16_t raw) {
    return (int16_t)raw * (4.096f / 32768.0f); // ±4.096V range
}

/**
 * @brief Read specified channel from ADS1115 ADC
 * 
 * @param channel ADC channel to read (0-3)
 * @return Raw 16-bit ADC value
 */
uint16_t read_ads_channel(int channel) {
    uint16_t config = (uint16_t) (ADS1115_BASE_CONFIG | (0x4000 | (channel << 12))); // Set MUX
    uint8_t config_bytes[2] = {config >> 8, config & 0xFF};
    i2c_write_with_retry(ADS1115_ADDR, config_bytes, 2, true, 4000);
    sleep_ms(8); // Wait for conversion
    uint8_t reg = 0x00; // Conversion register
    i2c_write_with_retry(ADS1115_ADDR, &reg, 1, true, 4000);
    uint8_t buffer[2];
    i2c_read_blocking(I2C_PORT, ADS1115_ADDR, buffer, 2, false);
    return (uint16_t)(buffer[0] << 8) | buffer[1];
}

/**
 * @brief Read data from all configured ADC channels
 */
/**
 * @brief Read calibration data from the BME280 sensor
 */
void read_bme280_calibration(void) {
    uint8_t calib_data[32];  // Total calibration data (24+8)
    uint8_t chip_id = 0;
    bool is_bmp280 = false;
    
    // Check which sensor we have
    uint8_t id_reg = 0xD0;
    if (mutex_enter_timeout_ms(&i2c_mutex, 50)) {
        i2c_write_with_retry(BME280_ADDR, &id_reg, 1, true, 5000);
        i2c_read_blocking(I2C_PORT, BME280_ADDR, &chip_id, 1, false);
        mutex_exit(&i2c_mutex);
    }
    
    is_bmp280 = (chip_id == 0x58);
    
    // Read temperature and pressure calibration (reg 0x88-0xA1)
    uint8_t reg = 0x88;
    bool success = false;
    
    if (mutex_enter_timeout_ms(&i2c_mutex, 50)) {
        i2c_write_with_retry(BME280_ADDR, &reg, 1, true, 100000);
        success = (i2c_read_blocking(I2C_PORT, BME280_ADDR, calib_data, 24, false) == 24);
        mutex_exit(&i2c_mutex);
    }
    
    if (!success) {
        if (DEBUG > 0) {
            printf("Failed to read BME/BMP280 calibration data (T/P)\r\n");
        }
        return;
    }
    
    // For BME280, read humidity calibration (reg 0xE1-0xE7)
    if (!is_bmp280) {
        reg = 0xE1;
        success = false;
        
        if (mutex_enter_timeout_ms(&i2c_mutex, 50)) {
            i2c_write_with_retry(BME280_ADDR, &reg, 1, true, 100000);
            success = (i2c_read_blocking(I2C_PORT, BME280_ADDR, calib_data + 24, 8, false) == 8);
            mutex_exit(&i2c_mutex);
        }
        
        if (!success && if DEBUG > 0) {
                printf("Failed to read BME280 calibration data (H)\r\n");
            }
            // Continue anyway - we can still read temperature and pressure
        }
    }
    
    // Parse temperature calibration
    bme280_calib.dig_T1 = (uint16_t)((calib_data[1] << 8) | calib_data[0]);
    bme280_calib.dig_T2 = (int16_t)((calib_data[3] << 8) | calib_data[2]);
    bme280_calib.dig_T3 = (int16_t)((calib_data[5] << 8) | calib_data[4]);
    
    // Parse pressure calibration
    bme280_calib.dig_P1 = (uint16_t)((calib_data[7] << 8) | calib_data[6]);
    bme280_calib.dig_P2 = (int16_t)((calib_data[9] << 8) | calib_data[8]);
    bme280_calib.dig_P3 = (int16_t)((calib_data[11] << 8) | calib_data[10]);
    bme280_calib.dig_P4 = (int16_t)((calib_data[13] << 8) | calib_data[12]);
    bme280_calib.dig_P5 = (int16_t)((calib_data[15] << 8) | calib_data[14]);
    bme280_calib.dig_P6 = (int16_t)((calib_data[17] << 8) | calib_data[16]);
    bme280_calib.dig_P7 = (int16_t)((calib_data[19] << 8) | calib_data[18]);
    bme280_calib.dig_P8 = (int16_t)((calib_data[21] << 8) | calib_data[20]);
    bme280_calib.dig_P9 = (int16_t)((calib_data[23] << 8) | calib_data[22]);
    
    // Parse humidity calibration (only for BME280)
    if (!is_bmp280) {
        bme280_calib.dig_H1 = calib_data[24];
        bme280_calib.dig_H2 = (int16_t)((calib_data[26] << 8) | calib_data[25]);
        bme280_calib.dig_H3 = calib_data[27];
        
        // These are packed differently
        bme280_calib.dig_H4 = (int16_t)((calib_data[28] << 4) | (calib_data[29] & 0x0F));
        bme280_calib.dig_H5 = (int16_t)((calib_data[30] << 4) | (calib_data[29] >> 4));
        bme280_calib.dig_H6 = (int8_t)calib_data[31];
    } else {
        // Set humidity calibration data to 0 for BMP280
        bme280_calib.dig_H1 = 0;
        bme280_calib.dig_H2 = 0;
        bme280_calib.dig_H3 = 0;
        bme280_calib.dig_H4 = 0;
        bme280_calib.dig_H5 = 0;
        bme280_calib.dig_H6 = 0;
    }
    
    if (DEBUG > 0) {
        printf("BME/BMP280 calibration data read successfully\r\n");
        printf("T1=%u, T2=%d, T3=%d\r\n", bme280_calib.dig_T1, bme280_calib.dig_T2, bme280_calib.dig_T3);
        printf("P1=%u, P2=%d, P3=%d\r\n", bme280_calib.dig_P1, bme280_calib.dig_P2, bme280_calib.dig_P3);
        if (!is_bmp280) {
            printf("H1=%u, H2=%d, H3=%u, H4=%d, H5=%d, H6=%d\r\n", 
                  bme280_calib.dig_H1, bme280_calib.dig_H2, bme280_calib.dig_H3,
                  bme280_calib.dig_H4, bme280_calib.dig_H5, bme280_calib.dig_H6);
        }
    }
}

/**
 * @brief Compensate BME280 raw data to get temperature, pressure, and humidity
 * 
 * @param raw_data Raw data from sensor (8 bytes)
 * @param temperature Pointer to store compensated temperature (in °C)
 * @param pressure Pointer to store compensated pressure (in Pa)
 * @param humidity Pointer to store compensated humidity (in %RH)
 * @param altitude Pointer to store calculated altitude (in m)
 * @note Based on default compensation code from datasheet
 */

bool read_bme280_data(void) {
    if (!HAS_BME280) return false;
    
    uint8_t raw_data[8]; // Pressure (3), Temperature (3), Humidity (2)
    bool success = false;
    
    if (mutex_enter_timeout_ms(&i2c_mutex, 50)) {
        // Read raw data from registers 0xF7-0xFE
        uint8_t reg = 0xF7;
        
        if (i2c_write_with_retry(BME280_ADDR, &reg, 1, true, 100000) &&
            i2c_read_blocking(I2C_PORT, BME280_ADDR, raw_data, 8, false) == 8) {
            
            float temperature = 0.0f;
            float pressure = 0.0f;
            float humidity = 0.0f;
            float altitude = 0.0f;
                
            // Convert raw data to physical values
            compensate_bme280_data(raw_data, &temperature, &pressure, &humidity, &altitude);
                
            // Store values in the sensor_readings structure
            if (mutex_enter_timeout_ms(&sensor_readings.data_mutex, 50)) {
                sensor_readings.pressure = pressure;
                sensor_readings.temperature = temperature;
                sensor_readings.humidity = humidity;
                sensor_readings.altitude = altitude;
                    
                mutex_exit(&sensor_readings.data_mutex);
                success = true;
            }
        }
        
        mutex_exit(&i2c_mutex);
    }
    
    return success;
}

/**
 * @brief Read data from the QMC5883L magnetometer
 * 
 * @return true if read was successful, false otherwise
 */
bool read_qmc5883l_data(void) {
    if (!HAS_QMC5883L) return false;
    
    bool success = false;
    uint8_t buffer[6];
    uint8_t status_reg = 0x06; // Status register
    uint8_t status;
    
    if (mutex_enter_timeout_ms(&i2c_mutex, 50)) {
        // Check if data is ready
        i2c_write_with_retry(QMC5883L_ADDR, &status_reg, 1, true, 100000);
        if (i2c_read_blocking(I2C_PORT, QMC5883L_ADDR, &status, 1, false) == 1 && status & 0x01) { // Data ready bit (DRDY)
            // Read data registers (0x00-0x05)
            uint8_t data_reg = 0x00;
            i2c_write_with_retry(QMC5883L_ADDR, &data_reg, 1, true, 100000);
            if (i2c_read_blocking(I2C_PORT, QMC5883L_ADDR, buffer, 6, false) == 6 && mutex_enter_timeout_ms(&sensor_readings.data_mutex, 50)) {
                    // Data is in X, Y, Z order, each axis is 16-bit little-endian
                    sensor_readings.mag[0] = (int16_t)(buffer[0] | (buffer[1] << 8));
                    sensor_readings.mag[1] = (int16_t)(buffer[2] | (buffer[3] << 8));
                    sensor_readings.mag[2] = (int16_t)(buffer[4] | (buffer[5] << 8));
                    mutex_exit(&sensor_readings.data_mutex);
                    success = true;
            }
        }
        mutex_exit(&i2c_mutex);
    }
    
    return success;
}

/**
 * @brief Read data from the MPU6050 accelerometer and gyroscope
 * 
 * @return true if read was successful, false otherwise
 */
bool read_mpu6050_data(void) {
    if (!HAS_MPU6050) return false;
    
    uint8_t buffer[14] = {0};  // Initialize to prevent use of uninitialized values
    bool success = false;
    
    if (!mutex_try_enter(&i2c_mutex, NULL)) {
        return false;
    }
    
    // Read all data at once from registers 0x3B to 0x48
    // ACCEL_XOUT_H (0x3B) through GYRO_ZOUT_L (0x48)
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    if (i2c_write_with_retry(MPU6050_ADDR, &reg, 1, true, 5000) &&
        i2c_read_with_retry(MPU6050_ADDR, buffer, 14, false, 5000)) {
        success = true;
    }
    
    // Clear interrupt flag
    uint8_t int_status_reg = 0x3A;
    uint8_t int_status;
    i2c_write_with_retry(MPU6050_ADDR, &int_status_reg, 1, true, 5000);
    i2c_read_with_retry(MPU6050_ADDR, &int_status, 1, false, 5000);
    
    mutex_exit(&i2c_mutex);
    
    // Process data if read was successful
    if (success && mutex_enter_timeout_ms(&sensor_readings.data_mutex, 50)) {
        // Convert and store accelerometer data (signed 16-bit values)
        sensor_readings.accel[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
        sensor_readings.accel[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
        sensor_readings.accel[2] = (int16_t)((buffer[4] << 8) | buffer[5]);
        
        // Convert and store gyroscope data (signed 16-bit values)
        sensor_readings.gyro[0] = (int16_t)((buffer[8] << 8) | buffer[9]);
        sensor_readings.gyro[1] = (int16_t)((buffer[10] << 8) | buffer[11]);
        sensor_readings.gyro[2] = (int16_t)((buffer[12] << 8) | buffer[13]);
        
        // Print raw values to debug
        if (DEBUG > 1) {
            printf("MPU6050 raw: ax=%d, ay=%d, az=%d, gx=%d, gy=%d, gz=%d\r\n",
                  sensor_readings.accel[0], sensor_readings.accel[1], sensor_readings.accel[2],
                  sensor_readings.gyro[0], sensor_readings.gyro[1], sensor_readings.gyro[2]);
        }
        
        mutex_exit(&sensor_readings.data_mutex);
        return true;
    }
    
    if (DEBUG > 0 && !success) {
        printf("Failed to read MPU6050 data\r\n");
    }
    
    return false;
}

void compensate_bme280_data(const uint8_t *raw_data, float *temperature, float *pressure, float *humidity, float *altitude) {
    // Parse raw values (20-bit values shifted into 32-bit integers)
    int32_t raw_temp = (raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4);
    int32_t raw_press = (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4);
    int32_t raw_hum = (raw_data[6] << 8) | raw_data[7];
    
    // Temperature calculation (returns temperature in DegC)
    int32_t var1;
    int32_t var2;
    int32_t t_fine;
    var1 = (((raw_temp >> 3) - ((int32_t)bme280_calib.dig_T1 << 1)) * ((int32_t)bme280_calib.dig_T2)) >> 11;
    var2 = (((((raw_temp >> 4) - ((int32_t)bme280_calib.dig_T1)) * ((raw_temp >> 4) - ((int32_t)bme280_calib.dig_T1))) >> 12) * ((int32_t)bme280_calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    *temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;
    
    // Pressure calculation (returns pressure in Pa)
    int64_t p_var1;
    int64_t p_var2;
    int64_t p;

    p_var1 = ((int64_t)t_fine) - 128000;
    p_var2 = p_var1 * p_var1 * (int64_t)bme280_calib.dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)bme280_calib.dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)bme280_calib.dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)bme280_calib.dig_P3) >> 8) + ((p_var1 * (int64_t)bme280_calib.dig_P2) << 12);
    p_var1 = ((((int64_t)1) << 47) + p_var1) * ((int64_t)bme280_calib.dig_P1) >> 33;
    
    if (p_var1 == 0) {
        *pressure = 0; // Avoid division by zero
    } else {
        p = 1048576 - raw_press;
        p = (((p << 31) - p_var2) * 3125) / p_var1;
        p_var1 = (((int64_t)bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        p_var2 = (((int64_t)bme280_calib.dig_P8) * p) >> 19;
        p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)bme280_calib.dig_P7) << 4);
        *pressure = (float) p / 256.0f; // Convert to Pa
    }
    
    // Check chip ID to determine if we have a BME280 or BMP280
    uint8_t id_reg = 0xD0;
    uint8_t chip_id = 0;
    
    // Try to read chip ID only if mutex is available
    bool id_read_success = false;
    if (mutex_try_enter(&i2c_mutex, NULL)) {
        i2c_write_with_retry(BME280_ADDR, &id_reg, 1, true, 5000);
        if (i2c_read_with_retry(BME280_ADDR, &chip_id, 1, false, 5000)) {
            id_read_success = true;
        }
        mutex_exit(&i2c_mutex);
    }
    
    // For BMP280 (0x58), set humidity to 0 since it doesn't have humidity sensor
    if (!id_read_success || chip_id == 0x58) {
        *humidity = 0.0f;
    } else {
        // Humidity Calculation for BME280
        int32_t v_x1_u32r;
        v_x1_u32r = (t_fine - ((int32_t)76800));
        v_x1_u32r = (((((raw_hum << 14) - (((int32_t)bme280_calib.dig_H4) << 20) - (((int32_t)bme280_calib.dig_H5) * v_x1_u32r)) +
                     ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)bme280_calib.dig_H6)) >> 10) * (((v_x1_u32r *
                     ((int32_t)bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                     ((int32_t)bme280_calib.dig_H2) + 8192) >> 14));
        
        v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)bme280_calib.dig_H1)) >> 4));
        v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
        v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
        
        *humidity = (float)(v_x1_u32r >> 12) / 1024.0f;
        
        // Constrain humidity to valid range
        if (*humidity > 100.0f) *humidity = 100.0f;
        if (*humidity < 0.0f) *humidity = 0.0f;
    }
    
    // Calculate altitude using the international barometric formula
    // Sea level pressure is 101325 Pa
    if (*pressure > 0) {
        *altitude = 44330.0f * (1.0f - powf((*pressure / 101325.0f), 0.1903f));
    } else {
        *altitude = 0.0f;
    }
    
    if (DEBUG > 1) {
        printf("Raw: T=%ld, P=%ld\r\n", raw_temp, raw_press);
        printf("Calculated: T=%.2f°C, P=%.2fPa, Alt=%.2fm\r\n", 
              *temperature, *pressure, *altitude);
    }
}