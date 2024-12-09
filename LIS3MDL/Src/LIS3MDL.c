/**
  ******************************************************************************
  * @file    LIS3MDL.c
  * @brief   This file file provides code for LIS3MDL sensor
  ******************************************************************************
  */

#include "LIS3MDL.h"
#include "usart.h"

// Global variables
//uint8_t LIS3MDL_DEVICE_ADDRESS;

int16_t X_offset;
int16_t Y_offset;
int16_t Z_offset;

/**
  * @brief  Pointer to a function that reads data from an I2C slave device.
  *         This callback handles both single-byte and multi-byte read operations.
  * @param  slave_address: 7-bit I2C address of the slave device.
  * @param  register_address: register address to read from. For multi-byte reads, MSB is set to 1.
  * @param  bytes: pointer to the buffer where received data will be stored.
  * @param  num_of_bytes: number of bytes to read.
  * @retval 0 on success (ACK received), or 1 if an error occurs (NACK received).
  */
static uint8_t (* LIS3MDL_read)(uint8_t slave_address, uint8_t register_address,uint8_t bytes[], uint8_t num_of_bytes) = 0;

/**
  * @brief  Function for registering reading callback function.
  * @param  callback: function to be registered as callback function.
  * @retval None.
  */
void LIS3MDL_registerReadCallback(void *callback)
{
	if(callback != 0)
	{
		LIS3MDL_read = callback;
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
static uint8_t (* LIS3MDL_write)(uint8_t slave_address, uint8_t register_address, uint8_t bytes[], uint8_t num_of_bytes) = 0;

/**
  * @brief  Function for registering writing callback function.
  * @param  callback: function to be registered as callback function.
  * @retval None.
  */
void LIS3MDL_registerWriteCallback(void *callback)
{
	if(callback != 0)
	{
		LIS3MDL_write = callback;
	}
}

/**
  * @brief  Reads a single byte from the specified register on the LIS3MDL sensor.
  * @param  register_address: address of the register to read from on the LIS3MDL.
  * @retval The byte value read from the specified register.
  */
uint8_t LIS3MDL_read_byte(uint8_t register_address)
{
	uint8_t byte;
	LIS3MDL_read(LIS3MDL_DEVICE_ADDRESS, register_address, &byte, 1);
	return byte;
}

/**
  * @brief  Reads multiple bytes starting from the specified register on the LIS3MDL sensor.
  * @param  register_address: address of the register to start reading from on the LIS3MDL.
  * @param  data: pointer to the buffer where received data will be stored.
  * @param  length: number of bytes to read.
  * @retval None.
  */
void LIS3MDL_read_bytes(uint8_t register_address, uint8_t bytes[], uint8_t num_of_bytes)
{
	LIS3MDL_read(LIS3MDL_DEVICE_ADDRESS, register_address, bytes, num_of_bytes);
}

/**
  * @brief  Writes a single byte to the specified register on the LIS3MDL sensor.
  * @param  register_address: address of the register to write to on the LIS3MDL.
  * @param  byte: byte to be written to the register
  * @retval None.
  */
void LIS3MDL_write_byte(uint8_t register_address, uint8_t byte)
{
	LIS3MDL_write(LIS3MDL_DEVICE_ADDRESS, register_address, &byte, 1);
}

/**
  * @brief  Writes multiple bytes starting from the specified register on the LIS3MDL sensor.
  * @param  register_address: address of the register to start writing from on the LIS3MDL.
  * @param  data: pointer to the buffer that contains data to be written.
  * @param  length: number of bytes to write.
  * @retval None.
  */
void LIS3MDL_write_bytes(uint8_t register_address, uint8_t bytes[], uint8_t num_of_bytes)
{
	LIS3MDL_write(LIS3MDL_DEVICE_ADDRESS, register_address, bytes, num_of_bytes);
}

/**
  * @brief  Function for initializing the sensor LIS3MDL.
  * @retval None.
  */
void LIS3MDL_init()
{
	uint8_t who_am_i_value;

	/*if (!LIS3MDL_read(LIS3MDL_DEVICE_ADDRESS_H, LIS3MDL_WHO_AM_I_ADDRESS, &who_am_i_value, 1)) {
		LIS3MDL_DEVICE_ADDRESS = LIS3MDL_DEVICE_ADDRESS_H;
	}
	else if (!LIS3MDL_read(LIS3MDL_DEVICE_ADDRESS_L, LIS3MDL_WHO_AM_I_ADDRESS, &who_am_i_value, 1)) {
		LIS3MDL_DEVICE_ADDRESS = LIS3MDL_DEVICE_ADDRESS_L;
	}
	else
	{
		//USART_write("LIS3MDL Init failed.");/////////////////
		return;
	}*/

	if (who_am_i_value != LIS3MDL_WHO_AM_I_VALUE)
	{
		//USART_write("LIS3MDL Init failed.");/////////////////
		return;
	}

	LIS3MDL_set_registers();
	LIS3MDL_read_offsets();

	//USART_write("LIS3MDL Init successful.");/////////////////
}


/**
  * @brief  Function for setting control registers of the sensor LIS3MDL.
  * @retval None.
  */
void LIS3MDL_set_registers()
{
	uint8_t CTRL_REG_DEVICE[LIS3MDL_NUMBER_OF_CTRL_REG];

	/*
	 * TODO
	 * full-scale selection
	 */

	CTRL_REG_DEVICE[0] = 0b01011110;
	CTRL_REG_DEVICE[0] = 0b00100000;
	CTRL_REG_DEVICE[0] = 0b00000000;
	CTRL_REG_DEVICE[0] = 0b00001000;
	CTRL_REG_DEVICE[0] = 0b00000000;

	LIS3MDL_write_bytes(LIS3MDL_CTRL_REG1_ADDRESS, CTRL_REG_DEVICE, LIS3MDL_NUMBER_OF_CTRL_REG);
}

/**
  * @brief  Function for reading offsets of values from the sensor LIS3MDL.
  * 		Offsets are written to their respective global variables.
  * @retval None.
  */
void LIS3MDL_read_offsets()
{
	uint8_t LIS3MDL_OFFSET_REG_X[2];
	uint8_t LIS3MDL_OFFSET_REG_Y[2];
	uint8_t LIS3MDL_OFFSET_REG_Z[2];

	LIS3MDL_read_bytes(LIS3MDL_OFFSET_X_REG_L_M_ADDRESS, LIS3MDL_OFFSET_REG_X, LIS3MDL_NUMBER_OF_OFFSET_REG);
	LIS3MDL_read_bytes(LIS3MDL_OFFSET_Y_REG_L_M_ADDRESS, LIS3MDL_OFFSET_REG_Y, LIS3MDL_NUMBER_OF_OFFSET_REG);
	LIS3MDL_read_bytes(LIS3MDL_OFFSET_Z_REG_L_M_ADDRESS, LIS3MDL_OFFSET_REG_Z, LIS3MDL_NUMBER_OF_OFFSET_REG);

	X_offset = LIS3MDL_OFFSET_REG_X[1] << 8;
	X_offset |= LIS3MDL_OFFSET_REG_X[0];
	Y_offset = LIS3MDL_OFFSET_REG_Y[1] << 8;
	Y_offset |= LIS3MDL_OFFSET_REG_Y[0];
	Z_offset = LIS3MDL_OFFSET_REG_Z[1] << 8;
	Z_offset |= LIS3MDL_OFFSET_REG_Z[0];
}

/**
  * @brief  Function for receiving data from magnetic sensor of the LIS3MDL.
  * @param	mag_bytes: array where data will be written.
  * 		[0]: value in X axis
  * 		[1]: value in Y axis
  * 		[2]: value in Z axis
  * @retval None.
  */
void LIS3MDL_get_mag_bytes(float mag_bytes[])
{
	uint8_t OUT_X[LIS3MDL_NUMBER_OF_OUT_REG];
	uint8_t OUT_Y[LIS3MDL_NUMBER_OF_OUT_REG];
	uint8_t OUT_Z[LIS3MDL_NUMBER_OF_OUT_REG];

	/*int16_t X_value;
	int16_t Y_value;
	int16_t Z_value;*/

	LIS3MDL_read_bytes(LIS3MDL_OUT_X_L_ADDRESS, OUT_X, LIS3MDL_NUMBER_OF_OUT_REG);
	LIS3MDL_read_bytes(LIS3MDL_OUT_Y_L_ADDRESS, OUT_Y, LIS3MDL_NUMBER_OF_OUT_REG);
	LIS3MDL_read_bytes(LIS3MDL_OUT_Z_L_ADDRESS, OUT_Z, LIS3MDL_NUMBER_OF_OUT_REG);

	/*uint8_t sign_X = (OUT_X[1] >> 7) & 1;
	uint8_t sign_Y = (OUT_Y[1] >> 7) & 1;
	uint8_t sign_Z = (OUT_Z[1] >> 7) & 1;



	X_value = OUT_X[1] << 8;
	X_value |= OUT_X[0];
	Y_value = OUT_Y[1] << 8;
	Y_value |= OUT_Y[0];
	Z_value = OUT_Z[1] << 8;
	Z_value |= OUT_Z[0];*/

	mag_bytes[0] = LIS3MDL_OUT_to_float(OUT_X[1], OUT_X[0]);//X_value + X_offset;
	mag_bytes[1] = LIS3MDL_OUT_to_float(OUT_Y[1], OUT_Y[0]);//Y_value + Y_offset;
	mag_bytes[2] = LIS3MDL_OUT_to_float(OUT_Z[1], OUT_Z[0]);//Z_value + Z_offset;
}

/**
  * @brief  Converts two registers to 2's complement floating-point value.
  * @param	MSB: most significant byte of the two registers.
  * @param  LSB: least significant byte of the two registers.
  * @retval Value of the two registers as float.
  */
float LIS3MDL_OUT_to_float(uint8_t MSB, uint8_t LSB)
{
	// TODO: Vracia mi chyby ? Neviem skompilovat ??
	/* float value;
	 * uint8_t sign = (MSB >> 7) & 1;
	 *
	 * value = sign << (FLOAT_SIZE_IN_BITS - 1);
	 * value |= MSB << 8;
	 * value &= ~(1 << 16);
	 * value |= LSB;
	 *
	 */

	return 0.0;
}

