#include "max30205.h"

/**
 * @brief Initializes the MAX30205 sensor in its Power-On Reset Value.
 */
uint8_t MAX30205_Init(I2C_HandleTypeDef *hi2c) { //The i2c address must be given in the main.c
    // Writes 0x00 in the configuration register to ensure the sensor is active and in normal mode,
    // that is setting all the bits from D0 to D7 to 0.
    uint8_t config_value = 0x00; 
    if (HAL_I2C_Mem_Write(hi2c, MAX30205_I2C_ADDR, MAX30205_CONFIGURATION, 1, &config_value, 1, 100) != HAL_OK) {
        return 0; // Communication errorConfigures the accelerometer with specified ODR, Full Scale, and performance mode.
    }
    
    return 1; // Success
}

/**
 * @brief Reads the clinical temperature from the sensor and converts it to degrees Celsius.
 */
void MAX30205_Read_Temp(I2C_HandleTypeDef *hi2c, float *temperature) {
    uint8_t buffer[2]; // array to hold the 2 bytes read from the temperature register
    int16_t raw_temp; // variable to hold the combined 16-bit raw temperature value. Signed because the temperature can be negative.
    
    // Reads the temperature data from the MAX30205. 
    // Interrogates the sensor at the ADDR 
    HAL_I2C_Mem_Read(hi2c, MAX30205_I2C_ADDR, MAX30205_TEMPERATURE, 1, buffer, 2, 100);
    
    // Reconstruction of the 16-bit raw temperature value from the two bytes read (buffer):
    // Shifts by 8 bits the first byte (MSB) to the left and performs a bitwise OR with the second byte (LSB) to combine them into a single 16-bit value.
    raw_temp = (int16_t)((buffer[0] << 8) | buffer[1]);

    // Conversion of the raw temperature value to degrees Celsius, according to the datasheet:
    *temperature = (float)raw_temp * 0.00390625f; // Each bit corresponds to 0.00390625 °C (1/256 °C)
}