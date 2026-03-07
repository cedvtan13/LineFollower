/*
 * calibration.c  —  clockwise spin auto-calibration
 */

#include "calibration.h"
#include "motor.h"
#include "sensor.h"
#include "stm32f4xx_hal.h"

#define CAL_SPIN_MS    4500u
#define CAL_SPIN_SPEED 12

static uint32_t calSpinStart = 0;
static uint8_t calDone = 0;

void Calibration_Init(void)
{
    calState = CAL_IDLE;
    calSpinStart = 0;
    calDone = 0;
}

void Calibration_Start(void)
{
    Sensor_CalStart();
    Motor_Enable();
    Motor_SetSpeeds(CAL_SPIN_SPEED, -CAL_SPIN_SPEED);
    calSpinStart = HAL_GetTick();
    calDone = 0;
    calState = CAL_SPIN;
}

void Calibration_Update(void)
{
    if (calState == CAL_SPIN) {
        Sensor_CalUpdate();
        if ((HAL_GetTick() - calSpinStart) >= CAL_SPIN_MS) {
            Motor_Stop();
            Sensor_CalFinish();
            calState = CAL_IDLE;
            calDone = 1u;
        }
    } else {
        Sensor_ReadAll();
        linePosition = Sensor_ComputePosition();
    }
}

void Calibration_Abort(void)
{
    Motor_Stop();
    calState = CAL_IDLE;
    calSpinStart = 0;
    calDone = 0;
}

uint8_t Calibration_IsDone(void)
{
    uint8_t done = calDone;
    calDone = 0;
    return done;
}

uint32_t Calibration_TimeRemaining(void)
{
    uint32_t elapsed;
    if (calState != CAL_SPIN) return 0u;
    elapsed = HAL_GetTick() - calSpinStart;
    return (elapsed >= CAL_SPIN_MS) ? 0u : (CAL_SPIN_MS - elapsed);
}