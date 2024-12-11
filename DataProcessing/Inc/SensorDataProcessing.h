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

#endif /* INC_SENSORDATAPROCESSING_H_ */
