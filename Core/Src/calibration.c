/*
 * calibration.c  —  Auto-calibration: robot spins clockwise for CAL_SPIN_MS
 *                    while all 16 sensors record their min/max ADC values.
 *
 * CAL_SPIN_SPEED : clockwise pivot speed (%) — independent of pid.baseSpeed.
 * CAL_SPIN_MS    : total spin duration in milliseconds.
 */

#include "calibration.h"
#include "sensor.h"
#include "motor.h"
#include "stm32f4xx_hal.h"

/* ================================================================
   CONFIGURATION
   ================================================================ */

#define CAL_SPIN_MS     5000u   /* spin duration (ms)                        */
#define CAL_SPIN_SPEED    30    /* clockwise pivot speed (%) — fixed, not pid */

/* ================================================================
   PRIVATE STATE
   ================================================================ */

static uint32_t calSpinStart = 0;
static uint8_t  calDone      = 0;

/* ================================================================
   IMPLEMENTATION
   ================================================================ */

void Calibration_Init(void)
{
    calState     = CAL_IDLE;
    calSpinStart = 0;
    calDone      = 0;
}

void Calibration_Start(void)
{
    Sensor_CalStart();
    Motor_Enable();
    /* Clockwise pivot: left wheel forward, right wheel backward */
    Motor_SetSpeeds(CAL_SPIN_SPEED, -CAL_SPIN_SPEED);
    calSpinStart = HAL_GetTick();
    calDone      = 0;
    calState     = CAL_SPIN;
}

void Calibration_Update(void)
{
    if (calState == CAL_SPIN) {
        Sensor_CalUpdate();
        uint32_t elapsed = HAL_GetTick() - calSpinStart;
        if (elapsed >= CAL_SPIN_MS) {
            /* Time is up — stop motors and compute thresholds */
            Motor_Stop();
            Sensor_CalFinish();
            calState = CAL_IDLE;
            calDone  = 1;
        }
    } else {
        Sensor_ReadAll();   /* keep display live while idle */
    }
}

void Calibration_Abort(void)
{
    Motor_Stop();
    calState     = CAL_IDLE;
    calSpinStart = 0;
    calDone      = 0;
}

uint8_t Calibration_IsDone(void)
{
    if (calDone) {
        calDone = 0;
        return 1;
    }
    return 0;
}

uint32_t Calibration_TimeRemaining(void)
{
    if (calState != CAL_SPIN) return 0u;
    uint32_t elapsed = HAL_GetTick() - calSpinStart;
    return (elapsed >= CAL_SPIN_MS) ? 0u : (CAL_SPIN_MS - elapsed);
}
