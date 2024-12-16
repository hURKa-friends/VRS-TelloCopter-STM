/**
  ******************************************************************************
  * @file    LSM6DS0.h
  * @brief   This file contains all the function prototypes and custom macros for the LSM6DS0.c file.
  ******************************************************************************
  */

#ifndef LSM6DS0_INC_LSM6DS0_H_
#define LSM6DS0_INC_LSM6DS0_H_

#include "main.h"
#include "LSM6DS0_map.h"
#include <math.h>
#include <string.h>

typedef enum {
	LSM6DS0_DISCONNECTED = 0x00U,
	LSM6DS0_CONNECTED	 = 0x01U,
	LSM6DS0_INITIALIZED  = 0x04U,
	LSM6DS0_I2C_ERROR    = 0x08U,
	LSM6DS0_INIT_ERROR 	 = 0x10U
} LSM6DS0_state;

/* Special Macros END */
#define NUMBER_OF_CTRL_REG			3
#define GYRO_REG_COUNT				6
#define ACCL_REG_COUNT				6

// Gyro scaler macros
#define GYRO_FS_LOW					0x00U
#define GYRO_FS_MID					0x01U
#define GYRO_FS_HIGH				0x03U
#define GYRO_FS_VALUE_LOW			8.75
#define GYRO_FS_VALUE_MID			17.5
#define GYRO_FS_VALUE_HIGH			70.0

// Accl scaler macros
#define ACCL_FS_LOW					0x00U
#define ACCL_FS_MID_LOW				0x02U
#define ACCL_FS_MID_HIGH			0x03U
#define ACCL_FS_HIGH				0x01U
#define ACCL_FS_VALUE_LOW			0.061
#define ACCL_FS_VALUE_MID_LOW		0.122
#define ACCL_FS_VALUE_MID_HIGH		0.244
#define ACCL_FS_VALUE_HIGH			0.732
/* Special Macros END */

/* USER CODE BEGIN Prototypes */
uint8_t LSM6DS0_read_byte(uint8_t register_address);
void LSM6DS0_read_array(uint8_t register_address, uint8_t data[], uint8_t length);
void LSM6DS0_write_byte(uint8_t register_address, uint8_t byte_value);
void LSM6DS0_write_array(uint8_t register_address, uint8_t data[], uint8_t length);

void LSM6DS0_init(void *readCallback, void *writeCallback);
void LSM6DS0_init_registers();
uint8_t LSM6DS0_get_device_state(void);
void LSM6DS0_calibrate_gyro();

void LSM6DS0_get_accl(int16_t *rawAcclX, int16_t *rawAcclY, int16_t *rawAcclZ);
void LSM6DS0_get_gyro(int16_t *rawGyroX, int16_t *rawGyroY, int16_t *rawGyroZ);
float LSM6DS0_parse_accl_data(int16_t rawAccl);
float LSM6DS0_parse_gyro_data(int16_t rawGyro);
void LSM6DS0_get_gyro_calib(float gyroCalib[]);
/* USER CODE END Prototypes */

#endif /* LSM6DS0_INC_LSM6DS0_H_ */
