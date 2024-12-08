/**
  ******************************************************************************
  * @file    LIS3MDL_map.h
  * @brief   This file provides the register mapping and device addresses for the LIS3MDL sensor.
  ******************************************************************************
  */

#ifndef LIS3MDL_INC_LIS3MDL_MAP_H_
#define LIS3MDL_INC_LIS3MDL_MAP_H_

/* I2C sensor general defines BEGIN */
#define 	LIS3MDL_DEVICE_ADDRESS_H			(0x1E << 1)
#define 	LIS3MDL_DEVICE_ADDRESS_L			(0x1C << 1)
/* I2C sensor general defines END */

/* Register values BEGIN */
#define 	LIS3MDL_WHO_AM_I_VALUE				0x3D
/* Register values END */

/* Register address mapping BEGIN */
// 			RESERVED 							0x00->0x04	// -
#define 	LIS3MDL_OFFSET_X_REG_L_M_ADDRESS	0x05		// R/W
#define 	LIS3MDL_OFFSET_X_REG_H_M_ADDRESS	0x06		// R/W
#define 	LIS3MDL_OFFSET_Y_REG_L_M_ADDRESS	0x07		// R/W
#define 	LIS3MDL_OFFSET_Y_REG_H_M_ADDRESS	0x08		// R/W
#define 	LIS3MDL_OFFSET_Z_REG_L_M_ADDRESS	0x09		// R/W
#define 	LIS3MDL_OFFSET_Z_REG_H_M_ADDRESS	0x0A		// R/W
// 			RESERVED 							0x0B->0x0E	// -
#define 	LIS3MDL_WHO_AM_I_ADDRESS			0x0F		// R
// 			RESERVED 							0x10->0x1F	// -
#define 	LIS3MDL_CTRL_REG1_ADDRESS			0x20		// R/W
#define 	LIS3MDL_CTRL_REG2_ADDRESS			0x21		// R/W
#define 	LIS3MDL_CTRL_REG3_ADDRESS			0x22		// R/W
#define 	LIS3MDL_CTRL_REG4_ADDRESS			0x23		// R/W
#define 	LIS3MDL_CTRL_REG5_ADDRESS			0x24		// R/W
// 			RESERVED 							0x25->0x26	// -
#define 	LIS3MDL_STATUS_REG_ADDRESS			0x27		// R
#define 	LIS3MDL_OUT_X_L_ADDRESS				0x28		// R
#define 	LIS3MDL_OUT_X_H_ADDRESS				0x29		// R
#define 	LIS3MDL_OUT_Y_L_ADDRESS				0x2A		// R
#define 	LIS3MDL_OUT_Y_H_ADDRESS				0x2B		// R
#define 	LIS3MDL_OUT_Z_L_ADDRESS				0x2C		// R
#define 	LIS3MDL_OUT_Z_H_ADDRESS				0x2D		// R
#define 	LIS3MDL_TEMP_OUT_L_ADDRESS			0x2E		// R
#define 	LIS3MDL_TEMP_OUT_H_ADDRESS			0x2F		// R
#define 	LIS3MDL_INT_CFG_ADDRESS				0x30		// R/W
#define 	LIS3MDL_INT_SRC_ADDRESS				0x31		// R
#define 	LIS3MDL_INT_THS_L_ADDRESS			0x32		// R/W
#define 	LIS3MDL_INT_THS_H_ADDRESS			0x33		// R/W
/* Register address mapping END */

#endif /* LIS3MDL_INC_LIS3MDL_MAP_H_ */
