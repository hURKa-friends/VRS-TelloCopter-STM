/**
  ******************************************************************************
  * @file    LIS3MDL.h
  * @brief   This file contains all the function prototypes and custom macros for the LIS3MDL.c file.
  ******************************************************************************
  */

#ifndef LIS3MDL_INC_LIS3MDL_H_
#define LIS3MDL_INC_LIS3MDL_H_

#include "main.h"
#include "LIS3MDL_map.h"

typedef enum {
	LIS3MDL_DISCONNECTED = 0x00U,
	LIS3MDL_CONNECTED	 = 0x01U,
	LIS3MDL_INITIALIZED  = 0x04U,
	LIS3MDL_I2C_ERROR    = 0x08U,
	LIS3MDL_INIT_ERROR 	 = 0x10U
} LIS3MDL_state;

/* Special Macros END */
#define LIS3MDL_NUMBER_OF_CTRL_REG			5
#define LIS3MDL_NUMBER_OF_OFFSET_REG		6
#define LIS3MDL_NUMBER_OF_OUT_REG			6
#define FLOAT_SIZE_IN_BITS		   		   (8 * sizeof(float))
#define MAG_FS_LOW							0
#define MAG_FS_MID_LOW						1
#define MAG_FS_MID_HIGH						2
#define MAG_FS_HIGH							3
#define MAG_FS_VALUE_LOW					6842.0
#define MAG_FS_VALUE_MID_LOW				3421.0
#define MAG_FS_VALUE_MID_HIGH				2281.0
#define MAG_FS_VALUE_HIGH					1711.0
/* Special Macros END */

/* USER CODE BEGIN Prototypes */
void LIS3MDL_registerReadCallback(void *callback);
void LIS3MDL_registerWriteCallback(void *callback);

uint8_t LIS3MDL_read_byte(uint8_t register_address);
void LIS3MDL_read_array(uint8_t register_address, uint8_t* data, uint8_t length);
void LIS3MDL_write_byte(uint8_t register_address, uint8_t byte_value);
void LIS3MDL_write_array(uint8_t register_address, uint8_t* data, uint8_t length);

void LIS3MDL_init();
void LIS3MDL_init_registers();
uint8_t LIS3MDL_get_device_state(void);
void LIS3MDL_read_offsets();

void LIS3MDL_get_mag(int16_t *rawMagX, int16_t *rawMagY, int16_t *rawMagZ);
float LIS3MDL_parse_mag_data(int16_t rawMag);

/* USER CODE END Prototypes */



#endif /* LIS3MDL_INC_LIS3MDL_H_ */
