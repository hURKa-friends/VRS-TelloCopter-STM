/**
  ******************************************************************************
  * @file    LSM6DS0.h
  * @brief   This file contains all the function prototypes and custom macros for the LSM6DS0.c file.
  ******************************************************************************
  */

#ifndef LSM6DS0_INC_LSM6DS0_H_
#define LSM6DS0_INC_LSM6DS0_H_

#include "LSM6DS0_map.h"

// Custom macros
#define NUMBER_OF_CTRL_REG			3
#define NUMBER_OF_OUT_REG			2
#define FLOAT_SIZE_IN_BITS		   (8 * sizeof(float))

// Functions declaration
void LSM6DS0_init();
void LSM6DS0_get_accl_bytes(int16_t accl_bytes[]);
void LSM6DS0_get_gyro_bytes(int16_t gyro_bytes[]);



#endif /* LSM6DS0_INC_LSM6DS0_H_ */
