/*
 * b_dot.c
 *
 *  Created on: May 5, 2025
 *      Author: Rodri
 */

#include "b_dot.h"
#include "magnetometer_driver.h"
#include "main.h"

#define ALPHA 0.1f
#define K 1.0f

static float B_filtered_prev[3] = {0.0f, 0.0f, 0.0f};
static uint32_t prev_time = 0;

extern float exponentialFilter(float curr, float prev, float alpha);

// Now this function just computes m and returns it
bool ADCS_Bdot_Compute(float m[3])
{
    int16_t raw_mag[3];
    float B[3];
    MAG_ReadMagneticField(raw_mag);
    MAG_ConvertToTeslas(raw_mag, B);

    uint32_t current_time = HAL_GetTick();
    if (prev_time == 0)
    {
        prev_time = current_time;
        for (int i = 0; i < 3; ++i) {
            B_filtered_prev[i] = B[i];
            m[i] = 0.0f;
        }
        return false;
    }

    float delta_t = (current_time - prev_time) / 1000.0f;
    if (delta_t <= 0.0f) return false;

    float B_filtered[3], B_dot[3];
    for (int i = 0; i < 3; ++i)
    {
        B_filtered[i] = exponentialFilter(B[i], B_filtered_prev[i], ALPHA);
        B_dot[i] = (B_filtered[i] - B_filtered_prev[i]) / delta_t;
        m[i] = -K * B_dot[i];
        B_filtered_prev[i] = B_filtered[i];
    }

    prev_time = current_time;
    return true;
}
