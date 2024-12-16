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
    //currentAngles[0] =  atan2(acclData[0] , sqrt((acclData[1]*acclData[1])+(acclData[2]*acclData[2])));
    //currentAngles[1] =  atan2(acclData[1] , sqrt((acclData[0]*acclData[0])+(acclData[2]*acclData[2])));
    currentAngles[0] =  atan2(acclData[0] , acclData[2]);
	currentAngles[1] =  atan2(acclData[1] , acclData[2]);
    //currentAngles[2] =  atan2(sqrt((acclData[0]*acclData[0])+(acclData[1]*acclData[1]))/acclData[2]);
}

float yaw_fromMag(float magData[])
{
    return atan2(magData[0] , magData[1]);
}

float movingAvgFilter(float* array, uint8_t sampleCount)
{
	float valueSum = 0;
	for(int i = 0; i < sampleCount; i++)
	{
		valueSum += array[i];
	}
	float mean = valueSum / sampleCount;
	return mean;
}

float linInterpolation(float input, float inputLimLow, float inputLimHigh, float outputLimLow, float outputLimHigh) {

	float velocity;
	int8_t sign;

	if(input > 0)
	{
		sign = 1;
	}
	else
	{
		sign = -1;
	}

	input = input * sign;

	if (input < inputLimLow) {
		input = inputLimLow;
	}
	if (input > inputLimHigh) {
		input = inputLimHigh;
	}

	velocity = outputLimLow + ((input - inputLimLow) / (inputLimHigh - inputLimLow)) * (outputLimHigh - outputLimLow);
	return velocity * sign;
}

flip_state flipSensor(float gyroData[]) {
	if(gyroData[0] > 270) {
		return FRONTFLIP;
	} else if(gyroData[0] < -270){
		return BACKFLIP;
	} else if(gyroData[1] > 270) {
		return RIGHTFLIP;
	} else if(gyroData[1] < -270) {
		return LEFTFLIP;
	} else {
		return NOFLIP;
	}
}

float rad2deg(float rad)
{
	return (rad / M_PI) * 180.0;
}
