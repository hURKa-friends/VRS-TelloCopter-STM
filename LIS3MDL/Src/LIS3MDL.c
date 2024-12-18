/**
  ******************************************************************************
  * @file    LIS3MDL.c
  * @brief   This file file provides code for LIS3MDL sensor
  ******************************************************************************
  */

#include "LIS3MDL.h"

// Global variables
int16_t mag_offsets[3];
LIS3MDL_state LIS3MDL_deviceState = LIS3MDL_DISCONNECTED;
float mag_scaler;
float initial_mag[3];

/**
  * @brief  Pointer to a function that reads data from an I2C slave device.
  *         This callback handles both single-byte and multi-byte read operations.
  * @param  slave_address: 7-bit I2C address of the slave device.
  * @param  register_address: register address to read from. For multi-byte reads, MSB is set to 1.
  * @param  bytes: pointer to the buffer where received data will be stored.
  * @param  num_of_bytes: number of bytes to read.
  * @retval 0 on success (ACK received), or 1 if an error occurs (NACK received).
  */
static uint8_t (* I2C_ReadData)(uint8_t slave_address, uint8_t register_address,uint8_t bytes[], uint8_t num_of_bytes) = 0;

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
  * @brief  Reads a single byte from the specified register on the LIS3MDL sensor.
  * @param  register_address: address of the register to read from on the LIS3MDL.
  * @retval The byte value read from the specified register.
  */
uint8_t LIS3MDL_read_byte(uint8_t register_address)
{
	uint8_t i2cState = 0, byte = 0;
	i2cState = I2C_ReadData(LIS3MDL_DEVICE_ADDRESS, register_address, &byte, 1);
	if(i2cState != 0)
		LIS3MDL_deviceState = LIS3MDL_I2C_ERROR;
	return byte;
}

/**
  * @brief  Reads multiple bytes starting from the specified register on the LIS3MDL sensor.
  * @param  register_address: address of the register to start reading from on the LIS3MDL.
  * @param  data: pointer to the buffer where received data will be stored.
  * @param  length: number of bytes to read.
  * @retval None.
  */
void LIS3MDL_read_array(uint8_t register_address, uint8_t data[], uint8_t length)
{
	uint8_t i2cState = 0;
	i2cState = I2C_ReadData(LIS3MDL_DEVICE_ADDRESS, register_address, data, length);
	if(i2cState != 0)
		LIS3MDL_deviceState = LIS3MDL_I2C_ERROR;
}

/**
  * @brief  Writes a single byte to the specified register on the LIS3MDL sensor.
  * @param  register_address: address of the register to write to on the LIS3MDL.
  * @param  byte: byte to be written to the register
  * @retval None.
  */
void LIS3MDL_write_byte(uint8_t register_address, uint8_t byte)
{
	uint8_t i2cState = 0;
	i2cState = I2C_WriteData(LIS3MDL_DEVICE_ADDRESS, register_address, &byte, 1);
	if(i2cState != 0)
		LIS3MDL_deviceState = LIS3MDL_I2C_ERROR;
}

/**
  * @brief  Writes multiple bytes starting from the specified register on the LIS3MDL sensor.
  * @param  register_address: address of the register to start writing from on the LIS3MDL.
  * @param  data: pointer to the buffer that contains data to be written.
  * @param  length: number of bytes to write.
  * @retval None.
  */
void LIS3MDL_write_array(uint8_t register_address, uint8_t data[], uint8_t length)
{
	uint8_t i2cState = 0;
	i2cState = I2C_WriteData(LIS3MDL_DEVICE_ADDRESS, register_address, data, length);
	if(i2cState != 0)
		LIS3MDL_deviceState = LIS3MDL_I2C_ERROR;
}

/**
  * @brief  Function for initializing the sensor LIS3MDL.
  * @param  readCallback: callback function for I2C reading.
  * @param  writeCallback: callback function for I2C writing.
  * @retval None.
  */
void LIS3MDL_init(void *readCallback, void *writeCallback)
{
	// Register Generic Read Callback
	if(readCallback != 0)
		I2C_ReadData = readCallback;
	// Register Generic Write Callback
	if(writeCallback != 0)
		I2C_WriteData = writeCallback;

	// Check if correct device is connected
	if(LIS3MDL_read_byte(LIS3MDL_WHO_AM_I_ADDRESS) == LIS3MDL_WHO_AM_I_VALUE)
		LIS3MDL_deviceState = LIS3MDL_CONNECTED;

	if (LIS3MDL_deviceState != LIS3MDL_CONNECTED)
		return; // LIS3MDL Init failed.

	LL_mDelay(100);
	LIS3MDL_init_registers();
	LL_mDelay(100);
	LIS3MDL_read_offsets();
	LL_mDelay(100);
	LIS3MDL_set_init_mag();

	LIS3MDL_deviceState = LIS3MDL_INITIALIZED;
	return; // LIS3MDL Init success
}


/**
  * @brief  Function for setting control registers of the sensor LIS3MDL.
  * @retval None.
  */
void LIS3MDL_init_registers()
{
	LIS3MDL_write_byte(LIS3MDL_CTRL_REG1_ADDRESS,  0x5EU); // CTRL_REG1  = 0b0101 1110 (ODR 300, FAST_ODR enabled, High performance)
	LIS3MDL_write_byte(LIS3MDL_CTRL_REG2_ADDRESS,  0x60U); // CTRL_REG2  = 0b0110 0000 (16 gauss)
	LIS3MDL_write_byte(LIS3MDL_CTRL_REG3_ADDRESS,  0x00U); // CTRL_REG3  = 0b0000 0000 (Continuous mode)
	LIS3MDL_write_byte(LIS3MDL_CTRL_REG4_ADDRESS, 0x08U); // CTRL_REG4  = 0b0000 1000 (High performance)
	LIS3MDL_write_byte(LIS3MDL_CTRL_REG5_ADDRESS, 0x00U); // CTRL_REG5  = 0b0000 0000 (Default)

	LL_mDelay(1);

	/*
	 * Initialization of GYROSCOPE SCALER VALUE BASED ON SELECTED MEASURE RANGE
	 */
	uint8_t mag_full_scale = LIS3MDL_read_byte(LIS3MDL_CTRL_REG2_ADDRESS);

	mag_full_scale >>= 5;
	mag_full_scale &= ~(100);

	switch(mag_full_scale) {
	case MAG_FS_LOW:
		mag_scaler = MAG_FS_VALUE_LOW;

		break;
	case MAG_FS_MID_LOW:
		mag_scaler = MAG_FS_VALUE_MID_LOW;

		break;
	case MAG_FS_MID_HIGH:
		mag_scaler = MAG_FS_VALUE_MID_HIGH;

		break;
	case MAG_FS_HIGH:
		mag_scaler = MAG_FS_VALUE_HIGH;

		break;
	default:
		break;
	}
}

/**
  * @brief  Retrieves the current state of the LIS3MDL device.
  * @retval The current state of the device as a value of type `LIS3MDL_state`.
  * @note   The state indicates whether the device is connected, initialized, or has encountered an I2C error.
  */
uint8_t LIS3MDL_get_device_state(void)
{
	return LIS3MDL_deviceState;
}

/**
  * @brief  Returns the initial (reference) values from magnetic sensor.
  * @param	initialMag:	Pointer to an array where initial values will be stored
  * 					[0] - X axis
  * 					[1] - Y axis
  * 					[2] - Z axis
  * @retval None.
  */
void LIS3MDL_getInitialMag(float initialMag[]) {
	memcpy(initialMag, initial_mag, sizeof(initial_mag));
}

void LIS3MDL_set_init_mag() {
	const uint8_t num_of_samples = 20;
	int16_t rawInitMagX, rawInitMagY, rawInitMagZ;
	float magX = 0,magY = 0, magZ = 0;

	for (uint8_t i = 0; i < num_of_samples; i++) {
		LIS3MDL_get_mag(&rawInitMagX, &rawInitMagY, &rawInitMagZ);
		magX += LIS3MDL_parse_mag_data(rawInitMagX);
		magY += LIS3MDL_parse_mag_data(rawInitMagY);
		magZ += LIS3MDL_parse_mag_data(rawInitMagZ);
	}

	initial_mag[0] = magX / num_of_samples;
	initial_mag[1] = magY / num_of_samples;
	initial_mag[2] = magZ / num_of_samples;
}

/**
  * @brief  Function for reading offsets of values from the sensor LIS3MDL.
  * 		Offsets are written to their respective global variables.
  * @retval None.
  */
void LIS3MDL_read_offsets()
{
	uint8_t LIS3MDL_offsets[LIS3MDL_NUMBER_OF_OFFSET_REG];

	LIS3MDL_read_array(LIS3MDL_OFFSET_X_REG_L_M_ADDRESS, LIS3MDL_offsets, sizeof(LIS3MDL_offsets));

	mag_offsets[0] = (int16_t)(((uint16_t)(LIS3MDL_offsets[0]) << 0) |
						  	   ((uint16_t)(LIS3MDL_offsets[1]) << 8));
	mag_offsets[1] = (int16_t)(((uint16_t)(LIS3MDL_offsets[2]) << 0) |
						  	   ((uint16_t)(LIS3MDL_offsets[3]) << 8));
	mag_offsets[2] = (int16_t)(((uint16_t)(LIS3MDL_offsets[4]) << 0) |
						  	   ((uint16_t)(LIS3MDL_offsets[5]) << 8));
}

/**
  * @brief  Reads raw accelerometer data from the LIS3MDL sensor.
  * @param  rawAcclX: Pointer to store the raw X-axis magnetic sensor data.
  * @param  rawAcclY: Pointer to store the raw Y-axis magnetic sensor data.
  * @param  rawAcclZ: Pointer to store the raw Z-axis magnetic sensor data.
  * @retval None
  */
void LIS3MDL_get_mag(int16_t *rawMagX, int16_t *rawMagY, int16_t *rawMagZ)
{
	uint8_t rawMagOut[LIS3MDL_NUMBER_OF_OUT_REG];
	LIS3MDL_read_array(LIS3MDL_OUT_X_L_ADDRESS, rawMagOut, sizeof(rawMagOut));

	*rawMagX = (int16_t)(((uint16_t)(rawMagOut[0]) << 0) |
						  ((uint16_t)(rawMagOut[1]) << 8)) - mag_offsets[0];
	*rawMagY = (int16_t)(((uint16_t)(rawMagOut[2]) << 0) |
						  ((uint16_t)(rawMagOut[3]) << 8)) - mag_offsets[1];
	*rawMagZ = (int16_t)(((uint16_t)(rawMagOut[4]) << 0) |
						  ((uint16_t)(rawMagOut[5]) << 8)) - mag_offsets[2];
}

/**
  * @brief  Converts raw magnetic sensor data to a scaled value in gauss.
  * @param  rawMag: The raw magnetic sensor data (16-bit value) read from the sensor.
  * @retval The magnetic sensor value scaled to gauss as a floating-point value.
  */
float LIS3MDL_parse_mag_data(int16_t rawMag)
{
	float magValue = ((float)rawMag) / mag_scaler;
	return magValue;
}
