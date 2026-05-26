#ifndef __MAX30205_DRIVER_H__
#define __MAX30205_DRIVER_H__

/**
 * \file MAX30205_driver.h
 * \brief Driver library for the MAX30205 IMU sensor.
 *
 * This header file provides the public interface for controlling the
 * MAX30205, including configuration and data reading functions.
 */

#include "stdint.h"
#include "main.h"
#include <stdint.h>

// --- I2C Communication Settings ---
// The I2C slave address of the MAX30205 is 0x6B (7-bit format).
// This is the address to be used with the HAL library functions.
#define MAX30205_I2C_ADDR 0x90 
#define MAX30205_TIMEOUT   100 // Timeout in milliseconds for I2C operations (lo teniamo?)

// --- Register Addresses ---
// These are the addresses of the key registers used to configure and read the MAX30205.
#define MAX30205_TEMPERATURE   0x00 // Temperature data register (16-bit) 
#define MAX30205_CONFIGURATION 0x01 // Configuration register
#define MAX30205_THYST         0x02 // Hysteresis register for temperature threshold
#define MAX30205_TOS           0x03 // Over-temperature shutdown threshold register


// --- Data Structures ---
// Data structures are unnecessary since the temperature information is stored in the
// corresponding register via int16_t to merge the byte and float to store the value in Celsius

// --- Public Function Prototypes ---
// The main API for the IMU driver.

/**
 * @brief Initializes the MAX30205 sensor.
 * @return 1 on success, 0 on failure.
 */
void MAX30205_Init();

/**
 * @brief Extracts the clinical temperature from the sensor and converts it to degrees Celsius.
 * @param hi2c Pointer to the I2C handle used for communication with the sensor
 * @param temperature Pointer to a float variable where the read temperature value will be stored in degrees Celsius.
 */
void MAX30205_Read_Temp(I2C_HandleTypeDef *hi2c, float *temperature);

#endif /* __MAX30205_DRIVER_H__ */
