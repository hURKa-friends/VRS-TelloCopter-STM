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
	I2C_ERROR    = 0x08U
} LSM6DS0_state;

/* Special Macros END */
#define NUMBER_OF_CTRL_REG			3
#define GYRO_REG_COUNT				6
#define ACCL_REG_COUNT				6
/* Special Macros END */

/* USER CODE BEGIN Prototypes */
uint8_t LSM6DS0_read_byte(uint8_t register_address);
void LSM6DS0_read_array(uint8_t register_address, uint8_t* data, uint8_t length);
void LSM6DS0_write_byte(uint8_t register_address, uint8_t byte_value);
void LSM6DS0_write_array(uint8_t register_address, uint8_t* data, uint8_t length);

void LSM6DS0_init(void *readCallback, void *writeCallback);
void LSM6DS0_init_registers();
uint8_t LSM6DS0_get_device_state(void);

void LSM6DS0_get_accl(uint16_t *rawAcclX, uint16_t *rawAcclY, uint16_t *rawAcclZ);
void LSM6DS0_get_gyro(uint16_t *rawGyroX, uint16_t *rawGyroY, uint16_t *rawGyroZ);
float LSM6DS0_parse_accl_data(uint16_t rawAccl);
float LSM6DS0_parse_gyro_data(uint16_t rawGyro);
/* USER CODE END Prototypes */

#endif /* LSM6DS0_INC_LSM6DS0_H_ */
