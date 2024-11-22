#include "LIS3MDL.h"
#include "i2c.h"
#include "usart.h"

uint8_t LIS3MDL_DEVICE_ADDRESS;

int16_t X_offset;
int16_t Y_offset;
int16_t Z_offset;

static uint8_t (* LIS3MDL_read)(uint8_t slave_address, uint8_t register_address, uint8_t num_of_bytes, uint8_t bytes[]) = 0;

/* Register callback */
void LIS3MDL_registerCallback_read_bytes(void *callback)
{
	if(callback != 0)
	{
		LIS3MDL_read = callback;
	}
}

static uint8_t (* LIS3MDL_write)(uint8_t slave_address, uint8_t register_address, uint8_t num_of_bytes, uint8_t bytes[]) = 0;

/* Register callback */
void LIS3MDL_registerCallback_write_bytes(void *callback)
{
	if(callback != 0)
	{
		LIS3MDL_write = callback;
	}
}

uint8_t LIS3MDL_read_byte(uint8_t register_address) {
	uint8_t byte;

	LIS3MDL_read(LIS3MDL_DEVICE_ADDRESS, register_address, &byte, 1);

	return byte;
}

void LIS3MDL_read_bytes(uint8_t register_address, uint8_t bytes[], uint8_t num_of_bytes) {
	LIS3MDL_read(LIS3MDL_DEVICE_ADDRESS, register_address, bytes, num_of_bytes);
}

void LIS3MDL_write_byte(uint8_t register_address, uint8_t byte) {
	LIS3MDL_write(LIS3MDL_DEVICE_ADDRESS, register_address, &byte, 1);
}

void LIS3MDL_write_bytes(uint8_t register_address, uint8_t bytes[], uint8_t num_of_bytes) {
	LIS3MDL_write(LIS3MDL_DEVICE_ADDRESS, register_address, bytes, num_of_bytes);
}

void LIS3MDL_init() {
	LIS3MDL_registerCallback_read_bytes(I2C_read());//////////////
	LIS3MDL_registerCallback_write_bytes(I2C_write());////////////

	uint8_t who_am_i_value;

	if (!LIS3MDL_read(LIS3MDL_DEVICE_ADDRESS_HIGH, WHO_AM_I_ADDRESS, &who_am_i_value, 1)) {
		LIS3MDL_DEVICE_ADDRESS = LIS3MDL_DEVICE_ADDRESS_HIGH;
	}
	else if (!LIS3MDL_read(LIS3MDL_DEVICE_ADDRESS_LOW, WHO_AM_I_ADDRESS, &who_am_i_value, 1)) {
		LIS3MDL_DEVICE_ADDRESS = LIS3MDL_DEVICE_ADDRESS_LOW;
	}
	else {
		USART_write("LIS3MDL Init failed.");/////////////////

		return;
	}

	if (who_am_i_value != WHO_AM_I_VALUE) {
		USART_write("LIS3MDL Init failed.");/////////////////

		return;
	}


	LIS3MDL_set_registers();
	LIS3MDL_read_offsets();

	USART_write("LIS3MDL Init successful.");/////////////////
}

void LIS3MDL_get_mag_bytes(int16_t mag_bytes[]) {
	uint8_t OUT_X[NUMBER_OF_OUT_REG];
	uint8_t OUT_Y[NUMBER_OF_OUT_REG];
	uint8_t OUT_Z[NUMBER_OF_OUT_REG];

	int16_t X_value;
	int16_t Y_value;
	int16_t Z_value;

	LIS3MDL_read_bytes(OUT_X_L_ADDRESS, OUT_X, NUMBER_OF_OUT_REG);
	LIS3MDL_read_bytes(OUT_Y_L_ADDRESS, OUT_Y, NUMBER_OF_OUT_REG);
	LIS3MDL_read_bytes(OUT_Z_L_ADDRESS, OUT_Z, NUMBER_OF_OUT_REG);

	X_value = OUT_X[1] << 8;
	X_value |= OUT_X[0];
	Y_value = OUT_Y[1] << 8;
	Y_value |= OUT_Y[0];
	Z_value = OUT_Z[1] << 8;
	Z_value |= OUT_Z[0];

	mag_bytes[0] = X_value + X_offset;
	mag_bytes[1] = Y_value + Y_offset;
	mag_bytes[2] = Z_value + Z_offset;
}

void LIS3MDL_set_registers() {
	uint8_t CTRL_REG_DEVICE[NUMBER_OF_CTRL_REG];

	LIS3MDL_read_bytes(CTRL_REG1_ADDRESS, CTRL_REG_DEVICE, NUMBER_OF_CTRL_REG);

	CTRL_REG_DEVICE[0] = 0b;
	CTRL_REG_DEVICE[0] = 0b;
	CTRL_REG_DEVICE[0] = 0b;
	CTRL_REG_DEVICE[0] = 0b;
	CTRL_REG_DEVICE[0] = 0b;

	LIS3MDL_write_bytes(CTRL_REG1_ADDRESS, CTRL_REG_DEVICE, NUMBER_OF_CTRL_REG);
}

void LIS3MDL_read_offsets() {
	uint8_t LIS3MDL_OFFSET_REG_X[2];
	uint8_t LIS3MDL_OFFSET_REG_Y[2];
	uint8_t LIS3MDL_OFFSET_REG_Z[2];

	LIS3MDL_read_bytes(OFFSET_X_REG_L_M_ADDRESS, LIS3MDL_offset_X, NUMBER_OF_OFFSET_REG);
	LIS3MDL_read_bytes(OFFSET_Y_REG_L_M_ADDRESS, LIS3MDL_offset_Y, NUMBER_OF_OFFSET_REG);
	LIS3MDL_read_bytes(OFFSET_Z_REG_L_M_ADDRESS, LIS3MDL_offset_Z, NUMBER_OF_OFFSET_REG);

	X_offset = LIS3MDL_OFFSET_REG_X[1] << 8;
	X_offset |= LIS3MDL_OFFSET_REG_X[0];
	Y_offset = LIS3MDL_OFFSET_REG_Y[1] << 8;
	Y_offset |= LIS3MDL_OFFSET_REG_Y[0];
	Z_offset = LIS3MDL_OFFSET_REG_Z[1] << 8;
	Z_offset |= LIS3MDL_OFFSET_REG_Z[0];
}
