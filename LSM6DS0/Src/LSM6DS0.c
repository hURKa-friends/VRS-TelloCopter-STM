/**
  ******************************************************************************
  * @file    LSM6DS0.c
  * @brief   This file file provides code for LSM6DS0 sensor
  ******************************************************************************
  */

#include "LSM6DS0.h"

// Global variables
LSM6DS0_state deviceState = DISCONNECTED;
float gyro_scaler, accl_scaler;

/**
  * @brief  Pointer to a function that reads data from an I2C slave device.
  *         This callback handles both single-byte and multi-byte read operations.
  * @param  slave_address: 7-bit I2C address of the slave device.
  * @param  register_address: register address to read from. For multi-byte reads, MSB is set to 1.
  * @param  bytes: pointer to the buffer where received data will be stored.
  * @param  num_of_bytes: number of bytes to read.
  * @retval 0 on success (ACK received), or 1 if an error occurs (NACK received).
  */
static uint8_t (* I2C_ReadData)(uint8_t slave_address, uint8_t register_address, uint8_t bytes[], uint8_t num_of_bytes) = 0;

/**
  * @brief  Pointer to a function that writes data to an I2C slave device.
  *         This callback handles both single-byte and multi-byte write operations.
  * @param  slave_address: 7-bit I2C address of the slave device.
  * @param  register_address: register address to read from. For multi-byte reads, MSB is set to 1.
  * @param  bytes: pointer to the buffer containing data to be transmitted.
  * @param  num_of_bytes: number of bytes to write.
  * @retval 0 on success (ACK received), or 1 if an error occurs (NACK received).
  */
static uint8_t (* I2C_WriteData)(uint8_t slave_address, uint8_t register_address, uint8_t bytes[], uint8_t num_of_bytes) = 0;

/**
  * @brief  Reads a single byte from the specified register on the LSM6DS0 sensor.
  * @param  register_address: address of the register to read from on the LSM6DS0.
  * @retval The byte value read from the specified register.
  */
uint8_t LSM6DS0_read_byte(uint8_t register_address)
{
	uint8_t i2cState = 0, byte = 0;
	i2cState = I2C_ReadData(LSM6DS0_DEVICE_ADDRESS, register_address, &byte, 1);
	if(i2cState != 0)
		deviceState = I2C_ERROR;
	return byte;
}

/**
  * @brief  Reads multiple bytes starting from the specified register on the LSM6DS0 sensor.
  * @param  register_address: address of the register to start reading from on the LSM6DS0.
  * @param  data: pointer to the buffer where received data will be stored.
  * @param  length: number of bytes to read.
  * @retval None.
  */
void LSM6DS0_read_bytes(uint8_t register_address, uint8_t bytes[], uint8_t num_of_bytes)
{
	uint8_t i2cState = 0;
	i2cState = I2C_ReadData(LSM6DS0_DEVICE_ADDRESS, register_address, bytes, num_of_bytes);
	if(i2cState != 0)
		deviceState = I2C_ERROR;
}

/**
  * @brief  Writes a single byte to the specified register on the LSM6DS0 sensor.
  * @param  register_address: address of the register to write to on the LSM6DS0.
  * @param  byte: byte to be written to the register
  * @retval None.
  */
void LSM6DS0_write_byte(uint8_t register_address, uint8_t byte)
{
	uint8_t i2cState = 0;
	i2cState = I2C_WriteData(LSM6DS0_DEVICE_ADDRESS, register_address, &byte, 1);
	if(i2cState != 0)
		deviceState = I2C_ERROR;
}

/**
  * @brief  Writes multiple bytes starting from the specified register on the LSM6DS0 sensor.
  * @param  register_address: address of the register to start writing from on the LSM6DS0.
  * @param  data: pointer to the buffer that contains data to be written.
  * @param  length: number of bytes to write.
  * @retval None.
  */
void LSM6DS0_write_bytes(uint8_t register_address, uint8_t bytes[], uint8_t num_of_bytes)
{
	uint8_t i2cState = 0;
	i2cState = I2C_WriteData(LSM6DS0_DEVICE_ADDRESS, register_address, bytes, num_of_bytes);
	if(i2cState != 0)
		deviceState = I2C_ERROR;
}

/**
  * @brief  Function for initializing the sensor LSM6DS0.
  * @retval None.
  */
void LSM6DS0_init(void *readCallback, void *writeCallback)
{
	// Register Generic Read Callback
	if(readCallback != 0)
		I2C_ReadData = readCallback;
	// Register Generic Write Callback
	if(writeCallback != 0)
		I2C_WriteData = writeCallback;

	// Check if correct device is connected
	if(LSM6DS0_read_byte(LSM6DS0_WHO_AM_I_ADDRESS) == LSM6DS0_WHO_AM_I_VALUE)
		deviceState = CONNECTED;

	if (deviceState != CONNECTED)
		return; // LSM6DS0 Init failed.

	LSM6DS0_init_registers();
	deviceState = INITIALIZED;
	return;  // LSM6DS0 Init succes.
}

/**
  * @brief  Function for setting control registers of the sensor LSM6DS0.
  * @retval None.
  */
void LSM6DS0_init_registers()
{
	uint8_t CTRL_REG_DEVICE[NUMBER_OF_CTRL_REG];  	// CTRL_REG8 0x22 -> CTRL_REG10 0x24
	uint8_t CTRL_REG_GYRO[NUMBER_OF_CTRL_REG];		// CTRL_REG1 0x10 -> CTRL_REG3 0x12
	uint8_t CTRL_REG_ACCL[NUMBER_OF_CTRL_REG];  	// CTRL_REG5 0x1F -> CTRL_REG7 0x21

	// SET SCALER FOR GYROSCOPE BASED ON SELECTED MEASURE RANGE
	uint8_t gyro_full_scale = LSM6DS0_read_byte(LSM6DS0_CTRL_REG1_G_ADDRESS);

	gyro_full_scale >>= 3;
	gyro_full_scale &= ~(11100);

	switch(gyro_full_scale) {
	case GYRO_FS_LOW:
		gyro_scaler = GYRO_FS_VALUE_LOW / GYRO_FS_VALUE_LOW;

		break;
	case GYRO_FS_MID:
		gyro_scaler = GYRO_FS_VALUE_LOW / GYRO_FS_VALUE_MID;

		break;
	case GYRO_FS_HIGH:
		gyro_scaler = GYRO_FS_VALUE_LOW / GYRO_FS_VALUE_HIGH;

		break;
	default:
		// N/A ?
		break;
	}

	// SET SCALER FOR ACCELEROMETER BASED ON SELECTED MEASURE RANGE
	uint8_t accl_full_scale = LSM6DS0_read_byte(LSM6DS0_CTRL_REG6_XL_ADDRESS);

	accl_full_scale >>= 3;
	accl_full_scale &= ~(11100);

	switch(accl_full_scale) {
	case ACCL_FS_LOW:
		accl_scaler = ACCL_FS_VALUE_LOW / ACCL_FS_VALUE_LOW;

		break;
	case ACCL_FS_MID_LOW:
		accl_scaler = ACCL_FS_VALUE_LOW / ACCL_FS_VALUE_MID_LOW;

		break;
	case ACCL_FS_MID_HIGH:
		accl_scaler = ACCL_FS_VALUE_LOW / ACCL_FS_VALUE_MID_HIGH;

		break;
	case ACCL_FS_HIGH:
		accl_scaler = ACCL_FS_VALUE_LOW / ACCL_FS_VALUE_HIGH;

		break;
	default:
		break;
	}

	/**
	  ******************************************************************************
	  * TODO: LSM6DS0 CTRL_REG initialization such as:
	  * - decimation
	  * - full-scale selection
	  * - frequency cutoff, high/low pass filters,
	  * - ORIENT_CFG_G
	  ******************************************************************************
	  */

	CTRL_REG_DEVICE[0] = 0b01000000; // Set CTRL_REG8  0x22 = 0b0100 0000 (Block Data Update)
	CTRL_REG_DEVICE[1] = 0b00000000; // Set CTRL_REG9  0x23 = 0b0000 0000 (Default)
	CTRL_REG_DEVICE[2] = 0b00000000; // Set CTRL_REG10 0x24 = 0b0000 0000 (Default)

	CTRL_REG_GYRO[0] = 0b10000000;	 // Set CTRL_REG1G 0x10 = 0b1000 0010 (ODR 238Hz, Cutoff 14Hz, 245dsp)
	CTRL_REG_GYRO[1] = 0b00000000;	 // Set CTRL_REG2G 0x11 = 0b0000 0000 (Default)
	CTRL_REG_GYRO[2] = 0b00000000;	 // Set CTRL_REG3G 0x12 = 0b0000 0000 (Default)

	CTRL_REG_ACCL[0] = 0b01111000;	 // Set CTRL_REG5A 0x1F = 0b0111 1000 (Update per 2 samples, XYZ enable, )
	CTRL_REG_ACCL[1] = 0b10000000;	 // Set CTRL_REG6A 0x20 = 0b1001 1000 (ODR 238Hz, Range +-8G, Anti-alias 408Hz)
	CTRL_REG_ACCL[2] = 0b00000000;	 // Set CTRL_REG7A 0x21 = 0b0000 0000 (Default)

	LSM6DS0_write_bytes(LSM6DS0_CTRL_REG8_ADDRESS, CTRL_REG_DEVICE, NUMBER_OF_CTRL_REG);
	LSM6DS0_write_bytes(LSM6DS0_CTRL_REG1_G_ADDRESS, CTRL_REG_GYRO, NUMBER_OF_CTRL_REG);
	LSM6DS0_write_bytes(LSM6DS0_CTRL_REG5_XL_ADDRESS, CTRL_REG_ACCL, NUMBER_OF_CTRL_REG);
}

/**
  * @brief  Retrieves the current state of the LSM6DS0 device.
  * @retval The current state of the device as a value of type `LSM6DS0_state`.
  * @note   The state indicates whether the device is connected, initialized, or has encountered an I2C error.
  */
uint8_t LSM6DS0_get_device_state(void)
{
	return deviceState;
}

/**
  * @brief  Reads raw accelerometer data from the LSM6DS0 sensor.
  * @param  rawAcclX: Pointer to store the raw X-axis accelerometer data.
  * @param  rawAcclY: Pointer to store the raw Y-axis accelerometer data.
  * @param  rawAcclZ: Pointer to store the raw Z-axis accelerometer data.
  * @retval None
  */
void LSM6DS0_get_accl(uint16_t *rawAcclX, uint16_t *rawAcclY, uint16_t *rawAcclZ)
{
	uint8_t rawAcclOut[ACCL_REG_COUNT];
	LSM6DS0_read_bytes(LSM6DS0_OUT_X_L_G_ADDRESS, rawAcclOut, sizeof(rawAcclOut));

	*rawAcclX = (uint16_t)(((uint16_t)(rawAcclOut[0]) << 0) |
	 	 	   	   	   	   ((uint16_t)(rawAcclOut[1]) << 8));
	*rawAcclY = (uint16_t)(((uint16_t)(rawAcclOut[2]) << 0) |
	 	 	   	   	   	   ((uint16_t)(rawAcclOut[3]) << 8));
	*rawAcclZ = (uint16_t)(((uint16_t)(rawAcclOut[4]) << 0) |
	 	 	   	   	   	   ((uint16_t)(rawAcclOut[5]) << 8));
}

/**
  * @brief  Reads raw gyroscope data from the LSM6DS0 sensor.
  * @param  rawGyroX: Pointer to store the raw X-axis gyroscope data.
  * @param  rawGyroY: Pointer to store the raw Y-axis gyroscope data.
  * @param  rawGyroZ: Pointer to store the raw Z-axis gyroscope data.
  * @retval None
  */
void LSM6DS0_get_gyro(uint16_t *rawGyroX, uint16_t *rawGyroY, uint16_t *rawGyroZ)
{
	uint8_t rawGyroOut[GYRO_REG_COUNT];
	LSM6DS0_read_bytes(LSM6DS0_OUT_X_L_G_ADDRESS, rawGyroOut, sizeof(rawGyroOut));

	*rawGyroX = (uint16_t)(((uint16_t)(rawGyroOut[0]) << 0) |
	 	 	   	   	   	   ((uint16_t)(rawGyroOut[1]) << 8));
	*rawGyroY = (uint16_t)(((uint16_t)(rawGyroOut[2]) << 0) |
	 	 	   	   	   	   ((uint16_t)(rawGyroOut[3]) << 8));
	*rawGyroZ = (uint16_t)(((uint16_t)(rawGyroOut[4]) << 0) |
	 	 	   	   	   	   ((uint16_t)(rawGyroOut[5]) << 8));
}

/**
  * @brief TODO: Implement accl data parsing
  */
float LSM6DS0_parse_accl_data(uint16_t rawAccl)
{
	float acclValue = ((float)rawAccl) * accl_scaler;
	return acclValue;
}

/**
  * @brief  Converts raw gyroscope data to a scaled value in degrees per second.
  * @param  rawGyro: The raw gyroscope data (16-bit value) read from the sensor.
  * @retval The gyroscope value scaled to degrees per second as a floating-point value.
  */
float LSM6DS0_parse_gyro_data(uint16_t rawGyro)
{
	float toDegrees = 1000.0;	// conversion to degrees
	float gyroValue = ((((float)rawGyro) * gyro_scaler) / toDegrees);
	return gyroValue;
}
