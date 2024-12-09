/*
 * SensorDataProcessing.c
 *
 *  Created on: Dec 9, 2024
 *      Author: MarekS
 */

#include "SensorDataProcessing.h"

float calculate_angle(float current_angle, float angular_velocity, float dt)
{
    current_angle += angular_velocity * dt;
    return current_angle;
}

float update_angle(float current_angle)
{
    static uint32_t prev_time = 0;

    // Read the angular velocity
    /* TODO: NEPOUZIVAT VOLANIE FUNKCII POUZI ARGUMENT
     * float angular_velocity_x = LSM6DS0_readXGyroRegister(0.0);
     */

    // Get the current time in milliseconds
    uint32_t curr_time = LL_usGetTick();

    // Calculate the time difference in seconds
    float dt = (curr_time - prev_time) / 1000.0f;  // Convert ms to seconds

    float angle_x;

    if (dt > 0)  // Ensure no division by zero
    {
        /* TODO: NEPOUZIVAT VOLANIE FUNKCII POUZI ARGUMENT
         * angle_x = calculate_angle(current_angle, angular_velocity_x, dt);
         */
    }

    // Update the previous time
    prev_time = curr_time;
    return angle_x;
}

void formatAndSaveToCSV(float current_angle, float y, float z)
{
    // Buffers to hold formatted strings
    char forx[6];
    char fory[6];
    char forz[6];

    // Clear tx_data2 buffer
    //memset(tx_data2, '\0', sizeof(tx_data2));

    // Format each value with the specified precision
    snprintf(forx, sizeof(forx), "%.2f", current_angle);
    snprintf(fory, sizeof(fory), "%.2f", y);
    snprintf(forz, sizeof(forz), "%.2f", z);

    // Combine formatted values into tx_data2
    //snprintf((char *)tx_data2, sizeof(tx_data2), "%s,%s,%s\r", forx, fory, forz);

    // Send tx_data2 buffer via USART2
    // TODO: Implement USART
    // USART2_PutBuffer(tx_data2, strlen((char *)tx_data2)); // Use strlen for accurate size
}
