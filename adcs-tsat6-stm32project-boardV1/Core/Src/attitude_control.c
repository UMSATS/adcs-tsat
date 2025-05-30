/*  attitude_control.c
 *
 *  Closed-loop three-state attitude-control scheduler for CubeSat ADCS
 *  S1  – sample   (magnetorquers OFF, magnetometer ON) 100 ms
 *  S2  – actuate  (magnetorquers ON, magnetometer ignored) 1 200 ms
 *  S3  – decay    (magnetorquers OFF, magnetometer ignored) 100 ms
 *
 *  Author: Alexandr Yermakov
 *  ----------------------------------------------------------------------- */

#include "attitude_control.h"
#include "main.h"
#include "magnetometer_driver.h"
#include "Magnetorquers_driver.h"
#include "b_dot.h"

#define M_THRESHOLD 0.01f  // Or tune as needed


#include <stdio.h>

/* ------------------------------------------------------------------------- */
/*                              configuration                                */
/* ------------------------------------------------------------------------- */
#define S1_DURATION_MS   100u      /* 0.1 s  – matches 20 Hz data-rate window */
#define S2_DURATION_MS  1200u      /* 1.2 s  – chosen for ≤10 °/s tumble rate */
#define S3_DURATION_MS   100u      /* 0.1 s  – conservative core-decay time  */

#define ALPHA            0.20f     /* exponential filter coefficient (S1)   */

/* ------------------------------------------------------------------------- */
/*                              private types                                */
/* ------------------------------------------------------------------------- */
typedef enum
{
    STATE_S1_SAMPLE = 0,
    STATE_S2_ACTUATE,
    STATE_S3_DECAY
} State_t;

typedef struct
{
    int16_t raw[3];
    float   tesla[3];     /* filtered values */
} Sample_t;

/* ------------------------------------------------------------------------- */
/*                             private variables                             */
/* ------------------------------------------------------------------------- */
static State_t   s_state          = STATE_S1_SAMPLE;
static uint32_t  s_state_entry_ms = 0u;          /* HAL_GetTick() timestamp   */
static Sample_t  s_last_sample    = {0};

/* ------------------------------------------------------------------------- */
/*                           forward declarations                            */
/* ------------------------------------------------------------------------- */
static void state_s1_sample(void);
static void state_s2_actuate(void);
static void state_s3_decay(void);

/* ------------------------------------------------------------------------- */
/*                              public API                                   */
/* ------------------------------------------------------------------------- */
void AttitudeControl_Init(void)
{
    /* make sure magnetorquers are OFF before we start sampling              */
    Magnetorquer1_Off();
    Magnetorquer2_Off();
    Magnetorquer3_Off();

    /* take an initial measurement so we start with a meaningful value       */
    MAG_ReadMagneticField(s_last_sample.raw);
    MAG_ConvertToTeslas(s_last_sample.raw, s_last_sample.tesla);

    s_state          = STATE_S1_SAMPLE;
    s_state_entry_ms = HAL_GetTick();
}

void AttitudeControl_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - s_state_entry_ms;

    switch (s_state)
    {
        /* -------------------------------------------------  S1: SAMPLE  --- */
        case STATE_S1_SAMPLE:
            state_s1_sample();
            if (elapsed >= S1_DURATION_MS)
            {
                s_state          = STATE_S2_ACTUATE;
                s_state_entry_ms = now;
            }
            break;

        /* -------------------------------------------------  S2: ACTUATE --- */
        case STATE_S2_ACTUATE:
            state_s2_actuate();
            if (elapsed >= S2_DURATION_MS)
            {
                s_state          = STATE_S3_DECAY;
                s_state_entry_ms = now;
            }
            break;

        /* -------------------------------------------------  S3: DECAY   --- */
        case STATE_S3_DECAY:
            state_s3_decay();
            if (elapsed >= S3_DURATION_MS)
            {
                s_state          = STATE_S1_SAMPLE;
                s_state_entry_ms = now;
            }
            break;

        default:
            /* should never happen */
            s_state = STATE_S1_SAMPLE;
            s_state_entry_ms = now;
            break;
    }
}

/* ------------------------------------------------------------------------- */
/*                             state handlers                                */
/* ------------------------------------------------------------------------- */

/* ---------- S1: sample magnetic field ------------------------------------ */
static void state_s1_sample(void)
{
    /* Ensure torquers are OFF so the reading is not contaminated.           */
    Magnetorquer1_Off();
    Magnetorquer2_Off();
    Magnetorquer3_Off();

    /* Raw reading */
    MAG_ReadMagneticField(s_last_sample.raw);

    /* Convert and low-pass filter (simple 1-pole IIR)                       */
    float tmp[3];
    MAG_ConvertToTeslas(s_last_sample.raw, tmp);

    for (int i = 0; i < 3; ++i)
    {
        s_last_sample.tesla[i] = ALPHA * tmp[i] +
                                 (1.0f - ALPHA) * s_last_sample.tesla[i];
    }

    /* Debug printouts (remove in flight builds)                             */
    printf("[S1] B-field  X: %.6f  Y: %.6f  Z: %.6f (mT)\r\n",
           s_last_sample.tesla[0],
           s_last_sample.tesla[1],
           s_last_sample.tesla[2]);
}

/* ---------- S2: actuate based on last sample ----------------------------- */
static void state_s2_actuate(void)
{
    float m[3];
    if (!ADCS_Bdot_Compute(m))
        return;

    // Axis 1
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

    // Axis 2
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

    // Axis 3
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
}
/* ---------- S3: decay – torquers already OFF ----------------------------- */
static void state_s3_decay(void)
{
    /* Nothing to do – we simply wait for the ferromagnetic cores to reset.  */
    Magnetorquer1_Off();
    Magnetorquer2_Off();
    Magnetorquer3_Off();
}
