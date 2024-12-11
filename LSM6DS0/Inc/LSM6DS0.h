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

typedef enum {
	DISCONNECTED = 0x00U,
	CONNECTED	 = 0x01U,
	INITIALIZED  = 0x04U,
	I2C_ERROR    = 0x08U,
	INIT_ERROR 	 = 0x10U
} LSM6DS0_state;

/* Special Macros END */
#define NUMBER_OF_CTRL_REG			3
#define GYRO_REG_COUNT				6
#define ACCL_REG_COUNT				6

// Gyro scaler macros
#define GYRO_FS_LOW					0x00U
#define GYRO_FS_MID					0x01U
#define GYRO_FS_HIGH				0x03U
#define GYRO_FS_VALUE_LOW			245.0
#define GYRO_FS_VALUE_MID			500.0
#define GYRO_FS_VALUE_HIGH			2000.0

// Accl scaler macros
#define ACCL_FS_LOW					0x00U
#define ACCL_FS_MID_LOW				0x02U
#define ACCL_FS_MID_HIGH			0x03U
#define ACCL_FS_HIGH				0x01U
#define ACCL_FS_VALUE_LOW			2.0
#define ACCL_FS_VALUE_MID_LOW		4.0
#define ACCL_FS_VALUE_MID_HIGH		8.0
#define ACCL_FS_VALUE_HIGH			16.0
/* Special Macros END */

/* USER CODE BEGIN Prototypes */
uint8_t LSM6DS0_read_byte(uint8_t register_address);
void LSM6DS0_read_array(uint8_t register_address, uint8_t* data, uint8_t length);
void LSM6DS0_write_byte(uint8_t register_address, uint8_t byte_value);
void LSM6DS0_write_array(uint8_t register_address, uint8_t* data, uint8_t length);

void LSM6DS0_init(void *readCallback, void *writeCallback);
void LSM6DS0_init_registers();
uint8_t LSM6DS0_get_device_state(void);

void LSM6DS0_get_accl(int16_t *rawAcclX, int16_t *rawAcclY, int16_t *rawAcclZ);
void LSM6DS0_get_gyro(int16_t *rawGyroX, int16_t *rawGyroY, int16_t *rawGyroZ);
float LSM6DS0_parse_accl_data(int16_t rawAccl);
float LSM6DS0_parse_gyro_data(int16_t rawGyro);
/* USER CODE END Prototypes */

#endif /* LSM6DS0_INC_LSM6DS0_H_ */
