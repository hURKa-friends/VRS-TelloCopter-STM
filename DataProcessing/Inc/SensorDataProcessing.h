/*
 * SensorDataProcessing.h
 *
 *  Created on: Dec 9, 2024
 *      Author: MarekS
 */

#ifndef INC_SENSORDATAPROCESSING_H_
#define INC_SENSORDATAPROCESSING_H_

#include "main.h"

float calculate_angle(float current_angle, float angular_velocity, float dt);
float update_angle(float current_angle);
void formatAndSaveToCSV(float current_angle, float y, float z);

#endif /* INC_SENSORDATAPROCESSING_H_ */
