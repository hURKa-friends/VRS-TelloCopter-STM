/*
 * SensorDataProcessing.h
 *
 *  Created on: Dec 9, 2024
 *      Author: MarekS
 */

#ifndef INC_SENSORDATAPROCESSING_H_
#define INC_SENSORDATAPROCESSING_H_

#include "main.h"

void calculate_angles(float currentAngles[], float acclData[]);

float movingAvgFilter(float* array, uint8_t sampleCount);

float linInterpolation(float input, float inputLimLow, float inputLimHigh, float outputLimLow, float outputLimHigh);

float rad2deg(float rad);

void yaw_fromMag(float yaw, float magData[]);

typedef enum {
	FRONTFLIP = 0x00U,
	BACKFLIP	 = 0x01U,
	RIGHTFLIP  = 0x04U,
	LEFTFLIP    = 0x08U,
	NOFLIP 	 = 0x10U
} flip_state;

flip_state flipSensor(float gyroData[]);

#endif /* INC_SENSORDATAPROCESSING_H_ */
