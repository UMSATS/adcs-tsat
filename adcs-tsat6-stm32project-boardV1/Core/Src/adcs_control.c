/*
 * adcs_control.c
 *
 *  Created on: May 5, 2025
 *      Author: Rodri
 */

#include "adcs_control.h"
#include "magnetometer_driver.h"
#include "Magnetorquers_driver.h"
#include "main.h"

#define M_THRESHOLD 0.01f // Magnetic moment threshold (Am^2), tune this
#define ALPHA 0.1f        // Filter coefficient
#define K 1.0f            // Control gain, tune this

static float B_filtered_prev[3] = {0.0f, 0.0f, 0.0f};
static uint32_t prev_time = 0;

extern float exponentialFilter(float curr, float prev, float alpha);

void ADCS_Bdot_Init(void)
{
    Magnetorquers_Init();
}

void ADCS_Bdot_Update(void)
{
    int16_t raw_mag[3];
    float B[3];
    MAG_ReadMagneticField(raw_mag);
    MAG_ConvertToTeslas(raw_mag, B);

    uint32_t current_time = HAL_GetTick();
    if (prev_time == 0)
    {
        prev_time = current_time;
        B_filtered_prev[0] = B[0];
        B_filtered_prev[1] = B[1];
        B_filtered_prev[2] = B[2];
        return;
    }

    float delta_t = (current_time - prev_time) / 1000.0f; // Seconds
    if (delta_t <= 0.0f) return;

    // Filter magnetic field
    float B_filtered[3];
    B_filtered[0] = exponentialFilter(B[0], B_filtered_prev[0], ALPHA);
    B_filtered[1] = exponentialFilter(B[1], B_filtered_prev[1], ALPHA);
    B_filtered[2] = exponentialFilter(B[2], B_filtered_prev[2], ALPHA);

    // Compute derivative
    float B_dot[3];
    B_dot[0] = (B_filtered[0] - B_filtered_prev[0]) / delta_t;
    B_dot[1] = (B_filtered[1] - B_filtered_prev[1]) / delta_t;
    B_dot[2] = (B_filtered[2] - B_filtered_prev[2]) / delta_t;

    // Compute magnetic moment
    float m[3];
    m[0] = -K * B_dot[0];
    m[1] = -K * B_dot[1];
    m[2] = -K * B_dot[2];

    // Control magnetorquers (bang-bang)
    if (m[0] > M_THRESHOLD)
    {
        Magnetorquer1_Forward();
        Magnetorquer1_Full_Strength();
    }
    else if (m[0] < -M_THRESHOLD)
    {
        Magnetorquer1_Reverse();
        Magnetorquer1_Full_Strength();
    }
    else
    {
        Magnetorquer1_Off();
    }

    if (m[1] > M_THRESHOLD)
    {
        Magnetorquer2_Forward();
        Magnetorquer2_Full_Strength();
    }
    else if (m[1] < -M_THRESHOLD)
    {
        Magnetorquer2_Reverse();
        Magnetorquer2_Full_Strength();
    }
    else
    {
        Magnetorquer2_Off();
    }

    if (m[2] > M_THRESHOLD)
    {
        Magnetorquer3_Forward();
        Magnetorquer3_Full_Strength();
    }
    else if (m[2] < -M_THRESHOLD)
    {
        Magnetorquer3_Reverse();
        Magnetorquer3_Full_Strength();
    }
    else
    {
        Magnetorquer3_Off();
    }

    // Update previous values
    B_filtered_prev[0] = B_filtered[0];
    B_filtered_prev[1] = B_filtered[1];
    B_filtered_prev[2] = B_filtered[2];
    prev_time = current_time;
}
