/**
  ******************************************************************************
  * @file    LSM6DS0.c
  * @brief   This file file provides code for LSM6DS0 sensor
  ******************************************************************************
  */

#include "LSM6DS0.h"
#include "usart.h"

// Global variables
uint8_t LSM6DS0_DEVICE_ADDRESS;

/**
  * @brief  Pointer to a function that reads data from an I2C slave device.
  *         This callback handles both single-byte and multi-byte read operations.
  * @param  slave_address: 7-bit I2C address of the slave device.
  * @param  register_address: register address to read from. For multi-byte reads, MSB is set to 1.
  * @param  bytes: pointer to the buffer where received data will be stored.
  * @param  num_of_bytes: number of bytes to read.
  * @retval 0 on success (ACK received), or 1 if an error occurs (NACK received).
  */
static uint8_t (* LSM6DS0_read)(uint8_t slave_address, uint8_t register_address, uint8_t bytes[], uint8_t num_of_bytes) = 0;

/**
  * @brief  Function for registering reading callback function.
  * @param  callback: function to be registered as callback function.
  * @retval None.
  */
void LSM6DS0_registerReadCallback(void *callback)
{
	if(callback != 0)
	{
		LSM6DS0_read = callback;
	}
}

/**
  * @brief  Pointer to a function that writes data to an I2C slave device.
  *         This callback handles both single-byte and multi-byte write operations.
  * @param  slave_address: 7-bit I2C address of the slave device.
  * @param  register_address: register address to read from. For multi-byte reads, MSB is set to 1.
  * @param  bytes: pointer to the buffer containing data to be transmitted.
  * @param  num_of_bytes: number of bytes to write.
  * @retval 0 on success (ACK received), or 1 if an error occurs (NACK received).
  */
static uint8_t (* LSM6DS0_write)(uint8_t slave_address, uint8_t register_address, uint8_t bytes[], uint8_t num_of_bytes) = 0;

/**
  * @brief  Function for registering writing callback function.
  * @param  callback: function to be registered as callback function.
  * @retval None.
  */
void LSM6DS0_registerWriteCallback(void *callback)
{
	if(callback != 0)
	{
		LSM6DS0_write = callback;
	}
}

/**
  * @brief  Reads a single byte from the specified register on the LSM6DS0 sensor.
  * @param  register_address: address of the register to read from on the LSM6DS0.
  * @retval The byte value read from the specified register.
  */
uint8_t LSM6DS0_read_byte(uint8_t register_address)
{
	uint8_t byte;
	LSM6DS0_read(LSM6DS0_DEVICE_ADDRESS, register_address, &byte, 1);
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
	LSM6DS0_read(LSM6DS0_DEVICE_ADDRESS, register_address, bytes, num_of_bytes);
}

/**
  * @brief  Writes a single byte to the specified register on the LSM6DS0 sensor.
  * @param  register_address: address of the register to write to on the LSM6DS0.
  * @param  byte: byte to be written to the register
  * @retval None.
  */
void LSM6DS0_write_byte(uint8_t register_address, uint8_t byte)
{
	LSM6DS0_write(LSM6DS0_DEVICE_ADDRESS, register_address, &byte, 1);
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
	LSM6DS0_write(LSM6DS0_DEVICE_ADDRESS, register_address, bytes, num_of_bytes);
}

/**
  * @brief  Function for initializing the sensor LSM6DS0.
  * @retval None.
  */
void LSM6DS0_init()
{
	uint8_t who_am_i_value;

	if (!LSM6DS0_read(LSM6DS0_DEVICE_ADDRESS_H, LSM6DS0_WHO_AM_I_ADDRESS, &who_am_i_value, 1))
	{
		LSM6DS0_DEVICE_ADDRESS = LSM6DS0_DEVICE_ADDRESS_H;
	}
	else if (!LSM6DS0_read(LSM6DS0_DEVICE_ADDRESS_L, LSM6DS0_WHO_AM_I_ADDRESS, &who_am_i_value, 1))
	{
		LSM6DS0_DEVICE_ADDRESS = LSM6DS0_DEVICE_ADDRESS_L;
	}
	else
	{
		//USART_write("LSM6DS0 Init failed.");/////////////////
		return;
	}

	if (who_am_i_value != LSM6DS0_WHO_AM_I_VALUE)
	{
		//USART_write("LSM6DS0 Init failed.");/////////////////
		return;
	}

	LSM6DS0_set_registers();

	//USART_write("LSM6DS0 Init successful.");////////////////////
}

/**
  * @brief  Function for setting control registers of the sensor LSM6DS0.
  * @retval None.
  */
void LSM6DS0_set_registers()
{
	uint8_t CTRL_REG_DEVICE[NUMBER_OF_CTRL_REG];
	uint8_t CTRL_REG_GYRO[NUMBER_OF_CTRL_REG];
	uint8_t CTRL_REG_ACCL[NUMBER_OF_CTRL_REG];

	/*
	 * TODO
	 * decimation
	 * full-scale selection
	 * frequency cutoff, high/low pass filters,
	 * ORIENT_CFG_G
	 */

	CTRL_REG_DEVICE[0] = 0b11000000; // Úprava podľa Denisa 0xC0U
	CTRL_REG_DEVICE[1] = 0b00000000;
	CTRL_REG_DEVICE[2] = 0b00000000;

	CTRL_REG_GYRO[0] = 0b10000010;
	CTRL_REG_GYRO[1] = 0b00000000;
	CTRL_REG_GYRO[2] = 0b00000000;

	CTRL_REG_ACCL[0] = 0b01111000;
	CTRL_REG_ACCL[1] = 0b00010000;
	CTRL_REG_ACCL[2] = 0b11000100;

	LSM6DS0_write_bytes(LSM6DS0_CTRL_REG8_ADDRESS, CTRL_REG_DEVICE, NUMBER_OF_CTRL_REG);
	LSM6DS0_write_bytes(LSM6DS0_CTRL_REG1_G_ADDRESS, CTRL_REG_GYRO, NUMBER_OF_CTRL_REG);
	LSM6DS0_write_bytes(LSM6DS0_CTRL_REG5_XL_ADDRESS, CTRL_REG_ACCL, NUMBER_OF_CTRL_REG);
}

/**
  * @brief  Function for receiving data from acceleration sensor of the LSM6DS0.
  * @param	accl_bytes: array where data will be written.
  * 		[0]: value in X axis
  * 		[1]: value in Y axis
  * 		[2]: value in Z axis
  * @retval None.
  */
void LSM6DS0_get_accl_bytes(int16_t accl_bytes[])
{
	uint8_t OUT_X[NUMBER_OF_OUT_REG];
	uint8_t OUT_Y[NUMBER_OF_OUT_REG];
	uint8_t OUT_Z[NUMBER_OF_OUT_REG];

	LSM6DS0_read_bytes(LSM6DS0_OUT_X_L_XL_ADDRESS, OUT_X, NUMBER_OF_OUT_REG);
	LSM6DS0_read_bytes(LSM6DS0_OUT_Y_L_XL_ADDRESS, OUT_Y, NUMBER_OF_OUT_REG);
	LSM6DS0_read_bytes(LSM6DS0_OUT_Z_L_XL_ADDRESS, OUT_Z, NUMBER_OF_OUT_REG);

	accl_bytes[0] = LSM6DS0_OUT_to_float(OUT_X[1], OUT_X[0]);
	accl_bytes[1] = LSM6DS0_OUT_to_float(OUT_Y[1], OUT_Y[0]);
	accl_bytes[2] = LSM6DS0_OUT_to_float(OUT_Z[1], OUT_Z[0]);
}

/**
  * @brief  Function for receiving data from gyroscope sensor of the LSM6DS0.
  * @param	gyro_bytes: array where data will be written.
  * 		[0]: value in X axis
  * 		[1]: value in Y axis
  * 		[2]: value in Z axis
  * @retval None.
  */
void LSM6DS0_get_gyro_bytes(float gyro_bytes[])
{
	uint8_t OUT_X[NUMBER_OF_OUT_REG];
	uint8_t OUT_Y[NUMBER_OF_OUT_REG];
	uint8_t OUT_Z[NUMBER_OF_OUT_REG];

	LSM6DS0_read_bytes(LSM6DS0_OUT_X_L_G_ADDRESS, OUT_X, NUMBER_OF_OUT_REG);
	LSM6DS0_read_bytes(LSM6DS0_OUT_Y_L_G_ADDRESS, OUT_Y, NUMBER_OF_OUT_REG);
	LSM6DS0_read_bytes(LSM6DS0_OUT_Z_L_G_ADDRESS, OUT_Z, NUMBER_OF_OUT_REG);

	gyro_bytes[0] = LSM6DS0_OUT_to_float(OUT_X[1], OUT_X[0]);
	gyro_bytes[1] = LSM6DS0_OUT_to_float(OUT_Y[1], OUT_Y[0]);
	gyro_bytes[2] = LSM6DS0_OUT_to_float(OUT_Z[1], OUT_Z[0]);
}

float LSM6DS0_readXGyroRegister(float x)
{
	uint8_t temp[2];
	LSM6DS0_read_bytes(LSM6DS0_OUT_X_L_G_ADDRESS, temp, 2);

	int16_t gyro_x = (temp[1] << 8) | temp[0];

	float x_temp = (float) gyro_x;

	x = ((x_temp * 8.75) / 1000.0);

	return x;
}

float LSM6DS0_readYGyroRegister(float y)
{
	uint8_t tempy[2];
	LSM6DS0_read_bytes(LSM6DS0_OUT_Y_L_G_ADDRESS, tempy, 2);

	int16_t gyro_y = (tempy[1] << 8) | tempy[0];

	float y_temp = (float) gyro_y;
	y = ((y_temp * 8.75) / 1000.0);

	return y;
}

float LSM6DS0_readZGyroRegister(float z)
{
	uint8_t tempz[2];
	LSM6DS0_read_bytes(LSM6DS0_OUT_Z_L_G_ADDRESS, tempz, 2);

	int16_t gyro_z = (tempz[1] << 8) | tempz[0];

	float z_temp = (float) gyro_z;
	z = ((z_temp * 8.75) / 1000.0);

	return z;
}

float calculate_angle(float current_angle, float angular_velocity, float dt)
{
    current_angle += angular_velocity * dt;
    return current_angle;
}

float update_angle(float current_angle)
{
    static uint32_t prev_time = 0;

    // Read the angular velocity
    float angular_velocity_x = LSM6DS0_readXGyroRegister(0.0);

    // Get the current time in milliseconds
    uint32_t curr_time = LL_usGetTick();

    // Calculate the time difference in seconds
    float dt = (curr_time - prev_time) / 1000.0f;  // Convert ms to seconds

    float angle_x;

    if (dt > 0) {  // Ensure no division by zero
        angle_x = calculate_angle(current_angle, angular_velocity_x, dt);
    }

    // Update the previous time
    prev_time = curr_time;
    return angle_x;
}


/**
  * @brief  Converts two registers to 2's complement floating-point value.
  * @param	MSB: most significant byte of the two registers.
  * @param  LSB: least significant byte of the two registers.
  * @retval Value of the two registers as float.
  */
float LSM6DS0_OUT_to_float(uint8_t MSB, uint8_t LSB)
{
	float value;
	uint8_t sign = (MSB >> 7) & 1;

	// TODO: Vracia mi chyby ? Neviem skompilovat ??
	/*
	 * value = sign << (FLOAT_SIZE_IN_BITS - 1);
	 * value |= MSB << 8;
	 * value &= ~(1 << 16);
	 * value |= LSB;
	 *
	 */

	return value;
}
