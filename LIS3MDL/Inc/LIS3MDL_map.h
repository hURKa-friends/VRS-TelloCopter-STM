/**
  ******************************************************************************
  * @file    LIS3MDL_map.h
  * @brief   This file provides the register mapping and device addresses for the LIS3MDL sensor.
  ******************************************************************************
  */

#ifndef LIS3MDL_INC_LIS3MDL_MAP_H_
#define LIS3MDL_INC_LIS3MDL_MAP_H_

/* I2C sensor general defines BEGIN */
#define 	LIS3MDL_DEVICE_ADDRESS_W			0x3CU
#define 	LIS3MDL_DEVICE_ADDRESS_R			0x3DU
#define 	LIS3MDL_DEVICE_ADDRESS				LIS3MDL_DEVICE_ADDRESS_W
/* I2C sensor general defines END */

/* Register values BEGIN */
#define 	LIS3MDL_WHO_AM_I_VALUE				0x3DU
/* Register values END */

/* Register address mapping BEGIN */
// 			RESERVED 							0x00->0x04	// -
#define 	LIS3MDL_OFFSET_X_REG_L_M_ADDRESS	0x05U		// R/W
#define 	LIS3MDL_OFFSET_X_REG_H_M_ADDRESS	0x06U		// R/W
#define 	LIS3MDL_OFFSET_Y_REG_L_M_ADDRESS	0x07U		// R/W
#define 	LIS3MDL_OFFSET_Y_REG_H_M_ADDRESS	0x08U		// R/W
#define 	LIS3MDL_OFFSET_Z_REG_L_M_ADDRESS	0x09U		// R/W
#define 	LIS3MDL_OFFSET_Z_REG_H_M_ADDRESS	0x0AU		// R/W
// 			RESERVED 							0x0B->0x0E	// -
#define 	LIS3MDL_WHO_AM_I_ADDRESS			0x0FU		// R
// 			RESERVED 							0x10->0x1F	// -
#define 	LIS3MDL_CTRL_REG1_ADDRESS			0x20U		// R/W
#define 	LIS3MDL_CTRL_REG2_ADDRESS			0x21U		// R/W
#define 	LIS3MDL_CTRL_REG3_ADDRESS			0x22U		// R/W
#define 	LIS3MDL_CTRL_REG4_ADDRESS			0x23U		// R/W
#define 	LIS3MDL_CTRL_REG5_ADDRESS			0x24U		// R/W
// 			RESERVED 							0x25->0x26	// -
#define 	LIS3MDL_STATUS_REG_ADDRESS			0x27U		// R
#define 	LIS3MDL_OUT_X_L_ADDRESS				0x28U		// R
#define 	LIS3MDL_OUT_X_H_ADDRESS				0x29U		// R
#define 	LIS3MDL_OUT_Y_L_ADDRESS				0x2AU		// R
#define 	LIS3MDL_OUT_Y_H_ADDRESS				0x2BU		// R
#define 	LIS3MDL_OUT_Z_L_ADDRESS				0x2CU		// R
#define 	LIS3MDL_OUT_Z_H_ADDRESS				0x2DU		// R
#define 	LIS3MDL_TEMP_OUT_L_ADDRESS			0x2EU		// R
#define 	LIS3MDL_TEMP_OUT_H_ADDRESS			0x2FU		// R
#define 	LIS3MDL_INT_CFG_ADDRESS				0x30U		// R/W
#define 	LIS3MDL_INT_SRC_ADDRESS				0x31U		// R
#define 	LIS3MDL_INT_THS_L_ADDRESS			0x32U		// R/W
#define 	LIS3MDL_INT_THS_H_ADDRESS			0x33U		// R/W
/* Register address mapping END */

#endif /* LIS3MDL_INC_LIS3MDL_MAP_H_ */
