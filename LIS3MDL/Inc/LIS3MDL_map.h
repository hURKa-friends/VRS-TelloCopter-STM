/**
  ******************************************************************************
  * @file    LIS3MDL_map.h
  * @brief   This file provides the register mapping and device addresses for the LIS3MDL sensor.
  ******************************************************************************
  */

#ifndef LIS3MDL_INC_LIS3MDL_MAP_H_
#define LIS3MDL_INC_LIS3MDL_MAP_H_


// Register addresses
#define OFFSET_X_REG_L_M_ADDRESS	0x05
#define OFFSET_X_REG_H_M_ADDRESS	0x06
#define OFFSET_Y_REG_L_M_ADDRESS	0x07
#define OFFSET_Y_REG_H_M_ADDRESS	0x08
#define OFFSET_Z_REG_L_M_ADDRESS	0x09
#define OFFSET_Z_REG_H_M_ADDRESS	0x0A
#define WHO_AM_I_ADDRESS			0x0F
#define CTRL_REG1_ADDRESS			0x20
#define CTRL_REG2_ADDRESS			0x21
#define CTRL_REG3_ADDRESS			0x22
#define CTRL_REG4_ADDRESS			0x23
#define CTRL_REG5_ADDRESS			0x24
#define STATUS_REG_ADDRESS			0x27
#define OUT_X_L_ADDRESS				0x28
#define OUT_X_H_ADDRESS				0x29
#define OUT_Y_L_ADDRESS				0x2A
#define OUT_Y_H_ADDRESS				0x2B
#define OUT_Z_L_ADDRESS				0x2C
#define OUT_Z_H_ADDRESS				0x2D
#define TEMP_OUT_L_ADDRESS			0x2E
#define TEMP_OUT_H_ADDRESS			0x2F
#define INT_CFG_ADDRESS				0x30
#define INT_SRC_ADDRESS				0x31
#define INT_THS_L_ADDRESS			0x32
#define INT_THS_H_ADDRESS			0x33

// Device values
#define WHO_AM_I_VALUE				0x3D
#define DEVICE_ADDRESS_HIGH		   (0x1E) << 1
#define DEVICE_ADDRESS_LOW		   (0x1C) << 1


#endif /* LIS3MDL_INC_LIS3MDL_MAP_H_ */
