/**
 * \file imu_driver.c
 * \brief Implementation file for the LSM6DSO16IS IMU driver.
 *
 * This file contains the low-level functions for communicating with the IMU
 * sensor via I2C, including register reads/writes and data processing.
 */

#include "MAX30101_driver.h"
#include "main.h"
#include "string.h"
#include "stdio.h"

extern I2C_HandleTypeDef hi2c3;

// --- Private Function Prototypes (Helper Functions) ---
// These functions are only used internally by the driver
static uint8_t sensor_read_register(uint8_t reg_addr, uint8_t* data, uint16_t data_len);
static uint8_t sensor_write_register(uint8_t reg_addr, uint8_t reg_data);
// --- Public Function Implementations ---

/**
 * @brief Verify that the MAX30101 is ON
 *
 * @return 1 on success, 0 on failure.
 */
uint8_t MAX30101_Init(void) {
    uint8_t pwr_rdy_value = 0;
    // Read the WHO_AM_I register and check if the returned value matches the expected one
    if (sensor_read_register(INTERRUPT_STATUS, &pwr_rdy_value, 1) && (INTERRUPT_STATUS_1 & 1)) {
        return 1; // Success
    }
    return 0; // Failure
}

/**
 * @brief Configures the sensor.
 * @param fifo_conf Configures FIFO settings.
 * @param mode_conf Configures sensor mode (HR/SpO2/Both).
 * @param spo2_conf Configures SPO2 settings.
 */
void MAX30101_Mode_Config(uint8_t fifo_conf, uint8_t mode_conf, uint8_t spo2_conf){
    // Combine ODR and FS into a single value for CTRL1_XL register

    sensor_write_register(FIFO_CONFIGURATION, fifo_conf);
    sensor_write_register(MODE_CONFIGURATION, mode_conf);
    sensor_write_register(SPO2_CONFIGURATION, spo2_conf);
}

/**
 * @brief Configures the sensor.
 * @param led_pa Configures LEDs pulse amplitude settings
 * @param multi_led Configures LEDs turn on-off stages 
 */
void MAX30101_LED_Config(uint8_t[4] led_pa, uint8_t[2] multi_led){
    for(int i = 0; i < 4; i++)
    {
        sensor_write_register((LED_PULSE_AMP + i), led_pa[i]);
    }
    for(int i = 0; i < 2; i++)
    {
        sensor_write_register((MULTI_LED_CTRL + i), multi_led[i]);
    }
}

/**
 * @brief Reads the raw and converted accelerometer data.
 * @param acc_data Pointer to an HEALTH_data struct where the results will be stored.
 * @param read_ptr Pointer to last read position of FIFO
 */
void MAX30101_Read_Data(HEALTH_data *acc_data, uint8_t read_ptr){
    //uint8_t raw_data[6]; // Buffer to hold 6 bytes for X, Y, Z axes
    int16_t x_raw, y_raw, z_raw;
    float sensitivity;

    // Read 6 bytes of data starting from the first accelerometer register.
    // The sensor auto-increments the register address, so a single read operation
    // fetches all 3 axes (X, Y, Z).
    sensor_read_register(IMU_ACC_OUT_X_L_REG, raw_data, 6);

    // Combine LSB and MSB to form a signed 16-bit integer for each axis
    x_raw = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    y_raw = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    z_raw = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    // Get the correct sensitivity factor based on the configured full scale
    sensitivity = get_accel_sensitivity(accelerometer_full_scale);

    // Convert raw data to 'g' (gravitational force) and store it in the struct
    acc_data->x = (float)x_raw * sensitivity;
    acc_data->y = (float)y_raw * sensitivity;
    acc_data->z = (float)z_raw * sensitivity;
}


// --- Private Function Implementations (Helper Functions) ---

/**
 * @brief Reads a specific number of bytes from the IMU's register via I2C.
 *
 * This is a low-level function that encapsulates the I2C read operation.
 * @param reg_addr The address of the register to read from.
 * @param data Pointer to the buffer where the read data will be stored.
 * @param data_len The number of bytes to read.
 * @return 1 on success, 0 on I2C communication error.
 */
static uint8_t sensor_read_register(uint8_t reg_addr, uint8_t* data, uint16_t data_len) {
    // Send the register address to the Sensor to prepare for reading.
    // This is a two-step process in I2C: first a write, then a read.
    if (HAL_I2C_Master_Transmit(&hi2c3, MAX3010_READ_ADDRESS << 1, &reg_addr, 1, MAX30101_TIMEOUT) != HAL_OK) {
        return 0; // Communication error
    }

    // Read the requested number of bytes from the IMU.
    if (HAL_I2C_Master_Receive(&hi2c3, MAX3010_READ_ADDRESS << 1, data, data_len, MAX30101_TIMEOUT) != HAL_OK) {
        return 0; // Communication error
    }
    return 1; // Success
}

/**
 * @brief Writes a single byte of data to a specific register on the IMU via I2C.
 *
 * @param reg_addr The address of the register to write to.
 * @param reg_data The byte of data to write.
 * @return 1 on success, 0 on I2C communication error.
 */
static uint8_t sensor_write_register(uint8_t reg_addr, uint8_t reg_data) {
    uint8_t tx_buffer[] = {reg_addr, reg_data};
    if (HAL_I2C_Master_Transmit(&hi2c3, MAX30101_WRITE_ADDRESS << 1, tx_buffer, sizeof(tx_buffer), MAX30101_TIMEOUT) != HAL_OK) {
        return 0; // Communication error
    }
    return 1; // Success
}
