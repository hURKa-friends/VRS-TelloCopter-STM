/**
  ******************************************************************************
  * @file    LIS3MDL.h
  * @brief   This file contains all the function prototypes and custom macros for the LIS3MDL.c file.
  ******************************************************************************
  */

#ifndef LIS3MDL_INC_LIS3MDL_H_
#define LIS3MDL_INC_LIS3MDL_H_

#include "LIS3MDL_map.h"

// Custom macros
#define NUMBER_OF_CTRL_REG			5
#define NUMBER_OF_OFFSET_REG		2
#define NUMBER_OF_OUT_REG			2

// Functions declaration
void LIS3MDL_init();
void LIS3MDL_get_mag_bytes(int16_t mag_bytes[]);



#endif /* LIS3MDL_INC_LIS3MDL_H_ */
