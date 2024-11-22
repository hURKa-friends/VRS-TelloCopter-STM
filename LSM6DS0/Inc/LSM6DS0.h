

#ifndef LSM6DS0_INC_LSM6DS0_H_
#define LSM6DS0_INC_LSM6DS0_H_

#include "LSM6DS0_map.h"

// CUSTOM VARIABLES
#define NUMBER_OF_CTRL_REG			3
#define NUMBER_OF_OUT_REG			2

// FUNCTIONS DECLARATION
void LSM6DS0_init();
void LSM6DS0_get_accl_bytes(int16_t accl_bytes[]);
void LSM6DS0_get_gyro_bytes(int16_t gyro_bytes[]);



#endif /* LSM6DS0_INC_LSM6DS0_H_ */
