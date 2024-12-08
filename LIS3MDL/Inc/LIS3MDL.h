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

/* Special Macros END */
#define LIS3MDL_NUMBER_OF_CTRL_REG			5
#define LIS3MDL_NUMBER_OF_OFFSET_REG		2
#define LIS3MDL_NUMBER_OF_OUT_REG			2
#define FLOAT_SIZE_IN_BITS		   (8 * sizeof(float))
/* Special Macros END */

/* USER CODE BEGIN Prototypes */
void LIS3MDL_registerReadCallback(void *callback);
void LIS3MDL_registerWriteCallback(void *callback);

uint8_t LIS3MDL_read_byte(uint8_t register_address);
void LIS3MDL_read_array(uint8_t register_address, uint8_t* data, uint8_t length);
void LIS3MDL_write_byte(uint8_t register_address, uint8_t byte_value);
void LIS3MDL_write_array(uint8_t register_address, uint8_t* data, uint8_t length);

void LIS3MDL_init();
void LIS3MDL_set_registers();
void LIS3MDL_read_offsets();

void LIS3MDL_get_mag_bytes(float mag_bytes[]);
float LIS3MDL_OUT_to_float(uint8_t MSB, uint8_t LSB);

/* USER CODE END Prototypes */



#endif /* LIS3MDL_INC_LIS3MDL_H_ */
