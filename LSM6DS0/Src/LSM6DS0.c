/**
  ******************************************************************************
  * @file    LSM6DS0.c
  * @brief   This file file provides code for LSM6DS0 sensor
  ******************************************************************************
  */

#include "LSM6DS0.h"

// Global variables
LSM6DS0_state LSM6DS0_deviceState = LSM6DS0_DISCONNECTED;
float gyro_scaler, accl_scaler;
float gyro_calibration[3];

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
	uint8_t	byte = 0;
	uint8_t i2cState = 0;
	i2cState = I2C_ReadData(LSM6DS0_DEVICE_ADDRESS, register_address, &byte, 1);
	if(i2cState != 0)
		LSM6DS0_deviceState = LSM6DS0_I2C_ERROR;
	return byte;
}

/**
  * @brief  Reads multiple bytes starting from the specified register on the LSM6DS0 sensor.
  * @param  register_address: address of the register to start reading from on the LSM6DS0.
  * @param  data: pointer to the buffer where received data will be stored.
  * @param  length: number of bytes to read.
  * @retval None.
  */
void LSM6DS0_read_array(uint8_t register_address, uint8_t data[], uint8_t length)
{
	uint8_t i2cState = 0;
	i2cState = I2C_ReadData(LSM6DS0_DEVICE_ADDRESS, register_address, data, length);
	if(i2cState != 0)
		LSM6DS0_deviceState = LSM6DS0_I2C_ERROR;
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
		LSM6DS0_deviceState = LSM6DS0_I2C_ERROR;
}

/**
  * @brief  Writes multiple bytes starting from the specified register on the LSM6DS0 sensor.
  * @param  register_address: address of the register to start writing from on the LSM6DS0.
  * @param  data: pointer to the buffer that contains data to be written.
  * @param  length: number of bytes to write.
  * @retval None.
  */
void LSM6DS0_write_array(uint8_t register_address, uint8_t data[], uint8_t length)
{
	uint8_t i2cState = 0;
	i2cState = I2C_WriteData(LSM6DS0_DEVICE_ADDRESS, register_address, data, length);
	if(i2cState != 0)
		LSM6DS0_deviceState = LSM6DS0_I2C_ERROR;
}

/**
  * @brief  Function for initializing the sensor LSM6DS0.
  * @param  readCallback: callback function for I2C reading.
  * @param  writeCallback: callback function for I2C writing.
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
		LSM6DS0_deviceState = LSM6DS0_CONNECTED;

	if (LSM6DS0_deviceState != LSM6DS0_CONNECTED)
		return; // LSM6DS0 Init failed.

	LSM6DS0_init_registers();

	LSM6DS0_calibrate_gyro();

	LSM6DS0_deviceState = LSM6DS0_INITIALIZED;
	return;  // LSM6DS0 Init success.
}

/**
  * @brief  Function for setting control registers of the sensor LSM6DS0.
  * @retval None.
  */
void LSM6DS0_init_registers()
{
	/**
	  ******************************************************************************
	  * TODO: LSM6DS0 CTRL_REG initialization such as:
	  * - decimation
	  * - full-scale selection
	  * - frequency cutoff, high/low pass filters,
	  * - ORIENT_CFG_G
	  ******************************************************************************
	  */

	LSM6DS0_write_byte(LSM6DS0_CTRL_REG1_G_ADDRESS,  0x80U); // CTRL_REG1  = 0b1000 0000 (ODR 238Hz, Cutoff 14Hz, 245dsp)
	LSM6DS0_write_byte(LSM6DS0_CTRL_REG2_G_ADDRESS,  0x00U); // CTRL_REG2  = 0b0000 0000 (Default)
	LSM6DS0_write_byte(LSM6DS0_CTRL_REG3_G_ADDRESS,  0x00U); // CTRL_REG3  = 0b0000 0000 (Default - HP filter off)
	LSM6DS0_write_byte(LSM6DS0_CTRL_REG4_ADDRESS,    0x38U); // CTRL_REG4  = 0b0011 1000 (XYZ Gyro output Enable)
	LSM6DS0_write_byte(LSM6DS0_CTRL_REG5_XL_ADDRESS, 0x38U); // CTRL_REG5  = 0b0011 1000 (XYZ Accl output Enable)
	LSM6DS0_write_byte(LSM6DS0_CTRL_REG6_XL_ADDRESS, 0x80U); // CTRL_REG6  = 0b1000 0000 (ODR 238Hz, Default)
	LSM6DS0_write_byte(LSM6DS0_CTRL_REG7_XL_ADDRESS, 0x00U); // CTRL_REG7  = 0b0000 0000 (Default)
	LSM6DS0_write_byte(LSM6DS0_CTRL_REG8_ADDRESS,    0x04U); // CTRL_REG8  = 0b0000 0100 (REG_ADDR automatic increment)
	LSM6DS0_write_byte(LSM6DS0_CTRL_REG9_ADDRESS,    0x00U); // CTRL_REG9  = 0b0000 0000 (Default)
	LSM6DS0_write_byte(LSM6DS0_CTRL_REG10_ADDRESS,   0x00U); // CTRL_REG10 = 0b0000 0000 (Default)

	LL_mDelay(1);

	/*
	 * Initialization of GYROSCOPE SCALER VALUE BASED ON SELECTED MEASURE RANGE
	 */
	uint8_t rawGyroScale = LSM6DS0_read_byte(LSM6DS0_CTRL_REG1_G_ADDRESS);
	rawGyroScale = rawGyroScale & 0x18U;
	rawGyroScale = rawGyroScale >> 3;

	switch(rawGyroScale)
	{
		case GYRO_FS_LOW:
			gyro_scaler = GYRO_FS_VALUE_LOW; break;
		case GYRO_FS_MID:
			gyro_scaler = GYRO_FS_VALUE_MID; break;
		case GYRO_FS_HIGH:
			gyro_scaler = GYRO_FS_VALUE_HIGH; break;
		default:
			LSM6DS0_deviceState = LSM6DS0_INIT_ERROR; break; // Undefined behaviour
	}

	/*
	 * Initialization of ACCELEROMETER SCALER VALUE BASED ON SELECTED MEASURE RANGE
	 */
	uint8_t rawAcclScale = LSM6DS0_read_byte(LSM6DS0_CTRL_REG6_XL_ADDRESS);
	rawAcclScale = rawAcclScale & 0x18U;
	rawAcclScale = rawAcclScale >> 3;

	switch(rawAcclScale)
	{
		case ACCL_FS_LOW:
			accl_scaler = ACCL_FS_VALUE_LOW; break;
		case ACCL_FS_MID_LOW:
			accl_scaler = ACCL_FS_VALUE_MID_LOW; break;
		case ACCL_FS_MID_HIGH:
			accl_scaler = ACCL_FS_VALUE_MID_HIGH; break;
		case ACCL_FS_HIGH:
			accl_scaler = ACCL_FS_VALUE_HIGH; break;
		default:
			LSM6DS0_deviceState = LSM6DS0_INIT_ERROR; break; // Undefined behaviour
	}
}

/**
  * @brief  Retrieves the current state of the LSM6DS0 device.
  * @retval The current state of the device as a value of type `LSM6DS0_state`.
  * @note   The state indicates whether the device is connected, initialized, or has encountered an I2C error.
  */
uint8_t LSM6DS0_get_device_state(void)
{
	return LSM6DS0_deviceState;
}

/**
  * @brief  Reads raw accelerometer data from the LSM6DS0 sensor.
  * @param  rawAcclX: Pointer to store the raw X-axis accelerometer data.
  * @param  rawAcclY: Pointer to store the raw Y-axis accelerometer data.
  * @param  rawAcclZ: Pointer to store the raw Z-axis accelerometer data.
  * @retval None
  */
void LSM6DS0_get_accl(int16_t *rawAcclX, int16_t *rawAcclY, int16_t *rawAcclZ)
{
	uint8_t rawAcclOut[ACCL_REG_COUNT];
	LSM6DS0_read_array(LSM6DS0_OUT_X_L_XL_ADDRESS, rawAcclOut, sizeof(rawAcclOut));

	*rawAcclX = (int16_t)(((uint16_t)(rawAcclOut[0]) << 0) |
	 	 	   	   	   	  ((uint16_t)(rawAcclOut[1]) << 8));
	*rawAcclY = (int16_t)(((uint16_t)(rawAcclOut[2]) << 0) |
	 	 	   	   	   	  ((uint16_t)(rawAcclOut[3]) << 8));
	*rawAcclZ = (int16_t)(((uint16_t)(rawAcclOut[4]) << 0) |
	 	 	   	   	   	  ((uint16_t)(rawAcclOut[5]) << 8));
}

/**
  * @brief  Reads raw gyroscope data from the LSM6DS0 sensor.
  * @param  rawGyroX: Pointer to store the raw X-axis gyroscope data.
  * @param  rawGyroY: Pointer to store the raw Y-axis gyroscope data.
  * @param  rawGyroZ: Pointer to store the raw Z-axis gyroscope data.
  * @retval None
  */
void LSM6DS0_get_gyro(int16_t *rawGyroX, int16_t *rawGyroY, int16_t *rawGyroZ)
{
	uint8_t rawGyroOut[GYRO_REG_COUNT];
	LSM6DS0_read_array(LSM6DS0_OUT_X_L_G_ADDRESS, rawGyroOut, sizeof(rawGyroOut));

	*rawGyroX = (int16_t)(((uint16_t)(rawGyroOut[0]) << 0) |
	 	 	   	   	   	  ((uint16_t)(rawGyroOut[1]) << 8));
	*rawGyroY = (int16_t)(((uint16_t)(rawGyroOut[2]) << 0) |
	 	 	   	   	   	  ((uint16_t)(rawGyroOut[3]) << 8));
	*rawGyroZ = (int16_t)(((uint16_t)(rawGyroOut[4]) << 0) |
	 	 	   	   	   	  ((uint16_t)(rawGyroOut[5]) << 8));
}

/**
  * @brief  Converts raw accelerometer data to a scaled value in g.
  * @param  rawAccl: The raw accelerometer data (16-bit value) read from the sensor.
  * @retval The accelerometer value scaled to g as a floating-point value.
  */
float LSM6DS0_parse_accl_data(int16_t rawAccl)
{
	float toG = 1000.0;	// conversion to g
	float acclValue = (((float)rawAccl) * accl_scaler) / toG;
	return acclValue;
}

/**
  * @brief  Converts raw gyroscope data to a scaled value in degrees per second.
  * @param  rawGyro: The raw gyroscope data (16-bit value) read from the sensor.
  * @retval The gyroscope value scaled to degrees per second as a floating-point value.
  */
float LSM6DS0_parse_gyro_data(int16_t rawGyro)
{
	float toDegrees = 1000.0;	// conversion to degrees
	float gyroValue = ((((float)rawGyro) * gyro_scaler) / toDegrees);
	return gyroValue;
}

void LSM6DS0_calibrate_gyro() {
	const uint8_t num_of_samples = 100;

	int16_t gyro_X[num_of_samples];
	int16_t gyro_Y[num_of_samples];
	int16_t gyro_Z[num_of_samples];
	float gyro_vel_X[num_of_samples];
	float gyro_vel_Y[num_of_samples];
	float gyro_vel_Z[num_of_samples];
	float gyro_X_min, gyro_X_max, gyro_Y_min, gyro_Y_max, gyro_Z_min, gyro_Z_max;

	for (uint8_t i = 0; i < num_of_samples; i++) {
		LSM6DS0_get_gyro(&gyro_X[i], &gyro_Y[i], &gyro_Z[i]);
		gyro_vel_X[i] = LSM6DS0_parse_gyro_data(gyro_X[i]);
		gyro_vel_Y[i] = LSM6DS0_parse_gyro_data(gyro_Y[i]);
		gyro_vel_Z[i] = LSM6DS0_parse_gyro_data(gyro_Z[i]);

		if (i == 0) {
			gyro_X_min = gyro_vel_X[i];
			gyro_X_max = gyro_vel_X[i];
			gyro_Y_min = gyro_vel_Y[i];
			gyro_Y_max = gyro_vel_Y[i];
			gyro_Z_min = gyro_vel_Z[i];
			gyro_Z_max = gyro_vel_Z[i];
		}

		if (gyro_vel_X[i] < gyro_X_min) {
			gyro_X_min = gyro_vel_X[i];
		}
		if (gyro_vel_X[i] > gyro_X_max) {
			gyro_X_max = gyro_vel_X[i];
		}
		if (gyro_vel_Y[i] < gyro_Y_min) {
			gyro_Y_min = gyro_vel_Y[i];
		}
		if (gyro_vel_Y[i] > gyro_Y_max) {
			gyro_Y_max = gyro_vel_Y[i];
		}
		if (gyro_vel_Z[i] < gyro_Z_min) {
			gyro_Z_min = gyro_vel_Z[i];
		}
		if (gyro_vel_Z[i] > gyro_Z_max) {
			gyro_Z_max = gyro_vel_Z[i];
		}

		LL_mDelay(round(1 / 238));
	}

	gyro_calibration[0] = (gyro_X_max + gyro_X_min) / 2;
	gyro_calibration[1] = (gyro_Y_max + gyro_Y_min) / 2;
	gyro_calibration[2] = (gyro_Z_max + gyro_Z_min) / 2;
}

void LSM6DS0_get_gyro_calib(float gyroCalib[]) {
	memcpy(gyroCalib, gyro_calibration, sizeof(gyro_calibration));
}
