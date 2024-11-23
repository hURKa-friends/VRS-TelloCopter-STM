/**
  ******************************************************************************
  * @file    LSM6DS0_map.h
  * @brief   This file provides the register mapping and device addresses for the LSM6DS0 sensor.
  ******************************************************************************
  */

#ifndef LSM6DS0_INC_LSM6DS0_MAP_H_
#define LSM6DS0_INC_LSM6DS0_MAP_H_


// Register addresses
#define ACT_THS_ADDRESS				0x04
#define ACT_DUR_ADDRESS				0x05
#define INT_GEN_CFG_XL_ADDRESS		0x06
#define INT_GEN_THS_X_XL_ADDRESS	0x07
#define INT_GEN_THS_Y_XL_ADDRESS	0x08
#define INT_GEN_THS_Z_XL_ADDRESS	0x09
#define INT_GEN_DUR_XL_ADDRESS		0x0A
#define REFERENCE_G_ADDRESS			0x0B
#define INT_CTRL_ADDRESS			0x0C
#define WHO_AM_I_ADDRESS			0x0F
#define CTRL_REG1_G_ADDRESS			0x10
#define CTRL_REG2_G_ADDRESS			0x11
#define CTRL_REG3_G_ADDRESS			0x12
#define ORIENT_CFG_G_ADDRESS		0x13
#define INT_GEN_SRC_G_ADDRESS		0x14
#define OUT_TEMP_L_ADDRESS			0x15
#define OUT_TEMP_H_ADDRESS			0x16
#define STATUS_REG_ADDRESS			0x17
#define OUT_X_L_G_ADDRESS			0x18
#define OUT_X_H_G_ADDRESS			0x19
#define OUT_Y_L_G_ADDRESS			0x1A
#define OUT_Y_H_G_ADDRESS			0x1B
#define OUT_Z_L_G_ADDRESS			0x1C
#define OUT_Z_H_G_ADDRESS			0x1D
#define CTRL_REG4_ADDRESS			0x1E
#define CTRL_REG5_XL_ADDRESS		0x1F
#define CTRL_REG6_XL_ADDRESS		0x20
#define CTRL_REG7_XL_ADDRESS		0x21
#define CTRL_REG8_ADDRESS			0x22
#define CTRL_REG9_ADDRESS			0x23
#define CTRL_REG10_ADDRESS			0x24
#define INT_GEN_SRC_XL_ADDRESS		0x26
#define STATUS_REG_ADDRESS			0x27
#define OUT_X_L_XL_ADDRESS			0x28
#define OUT_X_H_XL_ADDRESS			0x29
#define OUT_Y_L_XL_ADDRESS			0x2A
#define OUT_Y_H_XL_ADDRESS			0x2B
#define OUT_Z_L_XL_ADDRESS			0x2C
#define OUT_Z_H_XL_ADDRESS			0x2D
#define FIFO_CTRL_ADDRESS			0x2E
#define FIFO_SRC_ADDRESS			0x2F
#define INT_GEN_CFG_G_ADDRESS		0x30
#define INT_GEN_THS_XH_G_ADDRESS	0x31
#define INT_GEN_THS_XL_G_ADDRESS	0x32
#define INT_GEN_THS_YH_G_ADDRESS	0x33
#define INT_GEN_THS_YL_G_ADDRESS	0x34
#define INT_GEN_THS_ZH_G_ADDRESS	0x35
#define INT_GEN_THS_ZL_G_ADDRESS	0x36
#define INT_GEN_DUR_G_ADDRESS		0x37

// Device values
#define WHO_AM_I_VALUE				0x68
#define DEVICE_ADDRESS_HIGH 	   (0x6B) << 1
#define DEVICE_ADDRESS_LOW 		   (0x6A) << 1


#endif /* LSM6DS0_INC_LSM6DS0_MAP_H_ */
