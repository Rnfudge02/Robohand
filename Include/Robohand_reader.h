/*!
 * \file Robohand_reader.h
 * \brief Contains reader functions for sensor access.
 * \details Reader functions are called via the chosen backend.
 * \author Robert Fudge <rnfudge@mun.ca>
 * \date 2025
 * \copyright Apache 2.0 License
 */

#ifndef ROBOHAND_READER_H
#define ROBOHAND_READER_H

//Ensure C++ compatibility
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @defgroup sensor_readers Sensor Reader Functions
 * @brief Functions to read data from various sensors
 * @{
 */
uint16_t read_ads_channel(int channel);
float ads_voltage(uint16_t raw);
void read_bme280_calibration(void);
void compensate_bme280_data(const uint8_t* raw_data, float* temperature,
    float* pressure, float* humidity, float* altitude);
bool read_bme280_data(void);
bool read_mpu6050_data(void);
bool read_qmc5883l_data(void);
/** @} */

#ifdef __cplusplus
}
#endif

#endif // ROBOHAND_READER_H