/**
  ******************************************************************************
  * @file    LSM6DS0_map.h
  * @brief   This file provides the register mapping and device addresses for the LSM6DS0 sensor.
  ******************************************************************************
  */

#ifndef LSM6DS0_INC_LSM6DS0_MAP_H_
#define LSM6DS0_INC_LSM6DS0_MAP_H_

/* I2C sensor general defines BEGIN */
#define 	LSM6DS0_DEVICE_ADDRESS_W			0xD6U
#define 	LSM6DS0_DEVICE_ADDRESS_R			0xD7U
#define 	LSM6DS0_DEVICE_ADDRESS				LSM6DS0_DEVICE_ADDRESS_W
/* I2C sensor general defines END */

/* Register values BEGIN */
#define 	LSM6DS0_WHO_AM_I_VALUE				0x68
/* Register values END */

/* Register address mapping BEGIN */
// 			RESERVED 							0x00->0x03	// -
#define 	LSM6DS0_ACT_THS_ADDRESS				0x04		// R/W
#define 	LSM6DS0_ACT_DUR_ADDRESS				0x05		// R/W
#define 	LSM6DS0_INT_GEN_CFG_XL_ADDRESS		0x06		// R/W
#define 	LSM6DS0_INT_GEN_THS_X_XL_ADDRESS	0x07		// R/W
#define 	LSM6DS0_INT_GEN_THS_Y_XL_ADDRESS	0x08		// R/W
#define 	LSM6DS0_INT_GEN_THS_Z_XL_ADDRESS	0x09		// R/W
#define 	LSM6DS0_INT_GEN_DUR_XL_ADDRESS		0x0A		// R/W
#define 	LSM6DS0_REFERENCE_G_ADDRESS			0x0B		// R/W
#define 	LSM6DS0_INT_CTRL_ADDRESS			0x0C		// R/W
// 			RESERVED 							0x0D->0x0E	// -
#define 	LSM6DS0_WHO_AM_I_ADDRESS			0x0F		// R
#define 	LSM6DS0_CTRL_REG1_G_ADDRESS			0x10		// R/W
#define 	LSM6DS0_CTRL_REG2_G_ADDRESS			0x11		// R/W
#define 	LSM6DS0_CTRL_REG3_G_ADDRESS			0x12		// R/W
#define 	LSM6DS0_ORIENT_CFG_G_ADDRESS		0x13		// R/W
#define 	LSM6DS0_INT_GEN_SRC_G_ADDRESS		0x14		// R
#define 	LSM6DS0_OUT_TEMP_L_ADDRESS			0x15		// R
#define 	LSM6DS0_OUT_TEMP_H_ADDRESS			0x16		// R
#define 	LSM6DS0_STATUS_REG_ADDRESS			0x17		// R
#define 	LSM6DS0_OUT_X_L_G_ADDRESS			0x18		// R
#define 	LSM6DS0_OUT_X_H_G_ADDRESS			0x19		// R
#define 	LSM6DS0_OUT_Y_L_G_ADDRESS			0x1A		// R
#define 	LSM6DS0_OUT_Y_H_G_ADDRESS			0x1B		// R
#define 	LSM6DS0_OUT_Z_L_G_ADDRESS			0x1C		// R
#define 	LSM6DS0_OUT_Z_H_G_ADDRESS			0x1D		// R
#define 	LSM6DS0_CTRL_REG4_ADDRESS			0x1E		// R/W
#define 	LSM6DS0_CTRL_REG5_XL_ADDRESS		0x1F		// R/W
#define 	LSM6DS0_CTRL_REG6_XL_ADDRESS		0x20		// R/W
#define 	LSM6DS0_CTRL_REG7_XL_ADDRESS		0x21		// R/W
#define 	LSM6DS0_CTRL_REG8_ADDRESS			0x22		// R/W
#define 	LSM6DS0_CTRL_REG9_ADDRESS			0x23		// R/W
#define 	LSM6DS0_CTRL_REG10_ADDRESS			0x24		// R/W
// 			RESERVED 							0x25		// -
#define 	LSM6DS0_INT_GEN_SRC_XL_ADDRESS		0x26		// R
#define 	LSM6DS0_STATUS_REG_2_ADDRESS		0x27		// R
#define 	LSM6DS0_OUT_X_L_XL_ADDRESS			0x28		// R
#define 	LSM6DS0_OUT_X_H_XL_ADDRESS			0x29		// R
#define 	LSM6DS0_OUT_Y_L_XL_ADDRESS			0x2A		// R
#define 	LSM6DS0_OUT_Y_H_XL_ADDRESS			0x2B		// R
#define 	LSM6DS0_OUT_Z_L_XL_ADDRESS			0x2C		// R
#define 	LSM6DS0_OUT_Z_H_XL_ADDRESS			0x2D		// R
#define 	LSM6DS0_FIFO_CTRL_ADDRESS			0x2E		// R/W
#define 	LSM6DS0_FIFO_SRC_ADDRESS			0x2F		// R
#define 	LSM6DS0_INT_GEN_CFG_G_ADDRESS		0x30		// R/W
#define 	LSM6DS0_INT_GEN_THS_XH_G_ADDRESS	0x31		// R/W
#define 	LSM6DS0_INT_GEN_THS_XL_G_ADDRESS	0x32		// R/W
#define 	LSM6DS0_INT_GEN_THS_YH_G_ADDRESS	0x33		// R/W
#define 	LSM6DS0_INT_GEN_THS_YL_G_ADDRESS	0x34		// R/W
#define 	LSM6DS0_INT_GEN_THS_ZH_G_ADDRESS	0x35		// R/W
#define 	LSM6DS0_INT_GEN_THS_ZL_G_ADDRESS	0x36		// R/W
#define 	LSM6DS0_INT_GEN_DUR_G_ADDRESS		0x37		// R/W
// 			RESERVED 							0x38->0x7F	// -
/* Register address mapping END */

#endif /* LSM6DS0_INC_LSM6DS0_MAP_H_ */
