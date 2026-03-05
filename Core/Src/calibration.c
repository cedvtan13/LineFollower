/*
 * calibration.c  —  Auto-calibration for sensor thresholds
 */

#include "calibration.h"
#include "sensor.h"

/* ================================================================
   IMPLEMENTATION
   ================================================================ */

void Calibration_Init(void)
{
    calState   = CAL_IDLE;
    calibrated = 0;
}

void Calibration_Start(void)
{
    Sensor_CalStart();
    calState = CAL_SWEEP;
}

void Calibration_Update(void)
{
    if (calState == CAL_SWEEP) {
        Sensor_CalUpdate();
    } else {
        Sensor_ReadAll();
    }
}

void Calibration_Finish(void)
{
    Sensor_CalFinish();
    calState   = CAL_IDLE;
    calibrated = 1;
}

void Calibration_Abort(void)
{
    calState = CAL_IDLE;
}
