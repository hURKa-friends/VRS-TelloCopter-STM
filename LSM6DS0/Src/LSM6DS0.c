#include "LSM6DS0.h"
#include "i2c.h"
#include "usart.h"

uint8_t LSM6DS0_DEVICE_ADDRESS;

static uint8_t (* LSM6DS0_read)(uint8_t slave_address, uint8_t register_address, uint8_t num_of_bytes, uint8_t bytes[]) = 0;

/* Register callback */
void LSM6DS0_registerCallback_read_bytes(void *callback)
{
	if(callback != 0)
	{
		LSM6DS0_read = callback;
	}
}

static uint8_t (* LSM6DS0_write)(uint8_t slave_address, uint8_t register_address, uint8_t num_of_bytes, uint8_t bytes[]) = 0;

/* Register callback */
void LSM6DS0_registerCallback_write_bytes(void *callback)
{
	if(callback != 0)
	{
		LSM6DS0_write = callback;
	}
}

uint8_t LSM6DS0_read_byte(uint8_t register_address) {
	uint8_t byte;

	LSM6DS0_read(LSM6DS0_DEVICE_ADDRESS, register_address, &byte, 1);

	return byte;
}

void LSM6DS0_read_bytes(uint8_t register_address, uint8_t bytes[], uint8_t num_of_bytes) {
	LSM6DS0_read(LSM6DS0_DEVICE_ADDRESS, register_address, bytes, num_of_bytes);
}

void LSM6DS0_write_byte(uint8_t register_address, uint8_t byte) {
	LSM6DS0_write(LSM6DS0_DEVICE_ADDRESS, register_address, &byte, 1);
}

void LSM6DS0_write_bytes(uint8_t register_address, uint8_t bytes[], uint8_t num_of_bytes) {
	LSM6DS0_write(LSM6DS0_DEVICE_ADDRESS, register_address, bytes, num_of_bytes);
}

void LSM6DS0_init() {
	LSM6DS0_registerCallback_read_bytes(I2C_read());//////////////
	LSM6DS0_registerCallback_write_bytes(I2C_write());////////////

	uint8_t who_am_i_value;

	if (!LSM6DS0_read(LSM6DS0_DEVICE_ADDRESS_HIGH, WHO_AM_I_ADDRESS, &who_am_i_value, 1)) {
		LSM6DS0_DEVICE_ADDRESS = LSM6DS0_DEVICE_ADDRESS_HIGH;
	}
	else if (!LSM6DS0_read(LSM6DS0_DEVICE_ADDRESS_LOW, WHO_AM_I_ADDRESS, &who_am_i_value, 1)) {
		LSM6DS0_DEVICE_ADDRESS = LSM6DS0_DEVICE_ADDRESS_LOW;
	}
	else {
		USART_write("LSM6DS0 Init failed.");/////////////////

		return;
	}

	if (who_am_i_value != WHO_AM_I_VALUE) {
		USART_write("LSM6DS0 Init failed.");/////////////////

		return;
	}

	LSM6DS0_set_registers();

	USART_write("LSM6DS0 Init successful.");////////////////////
}

void LSM6DS0_get_accl_bytes(int16_t accl_bytes[]) {
	uint8_t OUT_X[NUMBER_OF_OUT_REG];
	uint8_t OUT_Y[NUMBER_OF_OUT_REG];
	uint8_t OUT_Z[NUMBER_OF_OUT_REG];

	int16_t X_value;
	int16_t Y_value;
	int16_t Z_value;

	LSM6DS0_read_bytes(OUT_X_L_XL_ADDRESS, OUT_X, NUMBER_OF_OUT_REG);
	LSM6DS0_read_bytes(OUT_Y_L_XL_ADDRESS, OUT_Y, NUMBER_OF_OUT_REG);
	LSM6DS0_read_bytes(OUT_Z_L_XL_ADDRESS, OUT_Z, NUMBER_OF_OUT_REG);

	X_value = OUT_X[1] << 8;
	X_value |= OUT_X[0];
	Y_value = OUT_Y[1] << 8;
	Y_value |= OUT_Y[0];
	Z_value = OUT_Z[1] << 8;
	Z_value |= OUT_Z[0];

	accl_bytes[0] = X_value;
	accl_bytes[1] = Y_value;
	accl_bytes[2] = Z_value;
}

void LSM6DS0_get_gyro_bytes(int16_t gyro_bytes[]) {
	uint8_t OUT_X[NUMBER_OF_OUT_REG];
	uint8_t OUT_Y[NUMBER_OF_OUT_REG];
	uint8_t OUT_Z[NUMBER_OF_OUT_REG];

	int16_t X_value;
	int16_t Y_value;
	int16_t Z_value;

	LSM6DS0_read_bytes(OUT_X_L_G_ADDRESS, OUT_X, NUMBER_OF_OUT_REG);
	LSM6DS0_read_bytes(OUT_Y_L_G_ADDRESS, OUT_Y, NUMBER_OF_OUT_REG);
	LSM6DS0_read_bytes(OUT_Z_L_G_ADDRESS, OUT_Z, NUMBER_OF_OUT_REG);

	X_value = OUT_X[1] << 8;
	X_value |= OUT_X[0];
	Y_value = OUT_Y[1] << 8;
	Y_value |= OUT_Y[0];
	Z_value = OUT_Z[1] << 8;
	Z_value |= OUT_Z[0];

	gyro_bytes[0] = X_value;
	gyro_bytes[1] = Y_value;
	gyro_bytes[2] = Z_value;
}

void LSM6DS0_set_registers() {
	uint8_t CTRL_REG_DEVICE[NUMBER_OF_CTRL_REG];
	uint8_t CTRL_REG_GYRO[NUMBER_OF_CTRL_REG];
	uint8_t CTRL_REG_ACCL[NUMBER_OF_CTRL_REG];

	LSM6DS0_read_bytes(CTRL_REG8_ADDRESS, CTRL_REG_DEVICE, NUMBER_OF_CTRL_REG);
	LSM6DS0_read_bytes(CTRL_REG1_G_ADDRESS, CTRL_REG_GYRO, NUMBER_OF_CTRL_REG);
	LSM6DS0_read_bytes(CTRL_REG5_XL_ADDRESS, CTRL_REG_ACCL, NUMBER_OF_CTRL_REG);

	CTRL_REG_DEVICE[0] = 0b;
	CTRL_REG_DEVICE[1] = 0b;
	CTRL_REG_DEVICE[2] = 0b;

	CTRL_REG_GYRO[0] = 0b;
	CTRL_REG_GYRO[1] = 0b;
	CTRL_REG_GYRO[2] = 0b;

	CTRL_REG_ACCL[0] = 0b;
	CTRL_REG_ACCL[1] = 0b;
	CTRL_REG_ACCL[2] = 0b;

	LSM6DS0_write_bytes(CTRL_REG8_ADDRESS, CTRL_REG_DEVICE, NUMBER_OF_CTRL_REG);
	LSM6DS0_write_bytes(CTRL_REG1_G_ADDRESS, CTRL_REG_GYRO, NUMBER_OF_CTRL_REG);
	LSM6DS0_write_bytes(CTRL_REG5_XL_ADDRESS, CTRL_REG_ACCL, NUMBER_OF_CTRL_REG);
}
