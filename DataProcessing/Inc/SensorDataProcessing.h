/*
 * SensorDataProcessing.h
 *
 *  Created on: Dec 9, 2024
 *      Author: MarekS
 */

#ifndef INC_SENSORDATAPROCESSING_H_
#define INC_SENSORDATAPROCESSING_H_

#include "main.h"

#define gyro_favoring			0.98

void calculate_angles(float currentAngles[], float acclData[]);

void complementaryFilter(float* complementaryAngle, float gyroAngle, float acclAngle);

float movingAvgFilter(float* array, uint8_t sampleCount);

float linInterpolation(float input, float inputLimLow, float inputLimHigh, float outputLimLow, float outputLimHigh);

float rad2deg(float rad);

float yaw_fromMag(float magData[], float lastAngle);

typedef enum {
	FRONTFLIP = 0x00U,
	BACKFLIP  = 0x01U,
	RIGHTFLIP = 0x04U,
	LEFTFLIP  = 0x08U,
	NOFLIP 	  = 0x10U
} flip_state;

flip_state flipSensor(float gyroData[]);

#endif /* INC_SENSORDATAPROCESSING_H_ */
