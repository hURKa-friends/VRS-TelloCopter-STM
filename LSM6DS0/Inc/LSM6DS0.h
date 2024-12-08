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

/* Special Macros END */
#define NUMBER_OF_CTRL_REG			3
#define NUMBER_OF_OUT_REG			2
#define FLOAT_SIZE_IN_BITS		   (8 * sizeof(float))
/* Special Macros END */

/* USER CODE BEGIN Prototypes */
void LSM6DS0_registerReadCallback(void *callback);
void LSM6DS0_registerWriteCallback(void *callback);

uint8_t LSM6DS0_read_byte(uint8_t register_address);
void LSM6DS0_read_array(uint8_t register_address, uint8_t* data, uint8_t length);
void LSM6DS0_write_byte(uint8_t register_address, uint8_t byte_value);
void LSM6DS0_write_array(uint8_t register_address, uint8_t* data, uint8_t length);

void LSM6DS0_init();
void LSM6DS0_set_registers();

void LSM6DS0_get_accl_bytes(int16_t accl_bytes[]);
void LSM6DS0_get_gyro_bytes(float gyro_bytes[]);

float LSM6DS0_readXGyroRegister(float x);
float LSM6DS0_readYGyroRegister(float y);
float LSM6DS0_readZGyroRegister(float z);

float calculate_angle(float current_angle, float angular_velocity, float dt);
float update_angle(float current_angle);

float LSM6DS0_OUT_to_float(uint8_t MSB, uint8_t LSB);
/* USER CODE END Prototypes */

#endif /* LSM6DS0_INC_LSM6DS0_H_ */
