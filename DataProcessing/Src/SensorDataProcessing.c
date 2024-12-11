/*
 * SensorDataProcessing.c
 *
 *  Created on: Dec 9, 2024
 *      Author: MarekS
 */

#include "SensorDataProcessing.h"
#include <math.h>

void calculate_angles(float currentAngles[], float acclData[])
{
    currentAngles[0] =  atan(acclData[0]/sqrt((acclData[1]*acclData[1])+(acclData[2]*acclData[2])));
    currentAngles[1] =  atan(acclData[1]/sqrt((acclData[0]*acclData[0])+(acclData[2]*acclData[2])));
    currentAngles[2] =  atan(sqrt((acclData[0]*acclData[0])+(acclData[1]*acclData[1]))/acclData[2]);
}

float movingAvgFilter(float* array, uint8_t sampleCount, uint8_t max) {
	float valueSum = 0;
	for(int i = 0; i < max; i++)
	{
		valueSum += array[i];
	}
	float mean = valueSum / max;
	return mean;
}

float linInterpolation(float input, float inputLimLow, float inputLimHigh, float outputLimLow, float outputLimHigh) {

	float velocity;
	float sign;

	if(input > 0) {
		sign = 1;
	} else {
		sign = -1;
	}

	if (input < inputLimLow) {
		input = inputLimLow;
	}
	if (input > inputLimHigh) {
		input = inputLimHigh;
	}

	velocity = outputLimLow + ((input - inputLimLow) / (inputLimHigh - inputLimLow)) * (outputLimHigh - outputLimLow);
	return velocity * sign;
}
