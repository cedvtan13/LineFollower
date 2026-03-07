/*
 * pid_controller.c  —  simple calibrated PD line follower with basic recovery
 */

#include "pid_controller.h"
#include "menu.h"
#include "motor.h"
#include "sensor.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#define RECOVERY_COAST_MS   100u
#define RECOVERY_SEARCH_MS  500u
#define RECOVERY_SPEED      22

static float pidLastError = 0.0f;
static int16_t recovL = 0;
static int16_t recovR = 0;
static int16_t lostPosition = 0;
static uint32_t lineLostAt = 0;

uint32_t runStartMs = 0;
uint32_t runElapsedMs = 0;
uint8_t pidLineLost = 0;

void PID_Reset(void)
{
    pidLastError = 0.0f;
    recovL = 0;
    recovR = 0;
    lostPosition = 0;
    lineLostAt = 0;
    runElapsedMs = 0;
    pidLineLost = 0;
}

void PID_Init(void)
{
    PID_Reset();
}

void PID_Update(void)
{
    static uint32_t lastSensorMs = 0;
    uint32_t now = HAL_GetTick();
    float error;
    float derivative;
    float output;
    float steering;
    float errNorm;
    float base;
    int16_t left;
    int16_t right;

    if ((now - lastSensorMs) >= 5u) {
        Sensor_ReadAll();
        lastSensorMs = now;
    }

    linePosition = Sensor_ComputePosition();
    if (runStartMs != 0u) runElapsedMs = now - runStartMs;

    if (sensorActiveCount == 0u) {
        pidLineLost = 1u;
        if (lineLostAt == 0u) {
            lineLostAt = now;
            lostPosition = linePosition;
            pidLastError = (float)lostPosition;
        }

        if ((now - lineLostAt) < RECOVERY_COAST_MS) {
            Motor_SetSpeeds(recovL, recovR);
        } else if ((now - lineLostAt) < (RECOVERY_COAST_MS + RECOVERY_SEARCH_MS)) {
            if (lostPosition >= 0)
                Motor_SetSpeeds(RECOVERY_SPEED, -RECOVERY_SPEED);
            else
                Motor_SetSpeeds(-RECOVERY_SPEED, RECOVERY_SPEED);
        } else {
            Motor_Stop();
        }
        return;
    }

    pidLineLost = 0u;
    lineLostAt = 0u;

    error = (float)linePosition;
    derivative = error - pidLastError;
    output = pid.Kp * error + pid.Kd * derivative;
    steering = output * (100.0f / (float)SENSOR_POS_MAX);
    errNorm = fabsf(error) / (float)SENSOR_POS_MAX;
    if (errNorm > 1.0f) errNorm = 1.0f;
    base = (float)pid.baseSpeed - ((float)(pid.baseSpeed - pid.turnSpeed) * errNorm * errNorm);

    left = (int16_t)(base + steering);
    right = (int16_t)(base - steering);

    if (left < -100) left = -100;
    if (left > 100) left = 100;
    if (right < -100) right = -100;
    if (right > 100) right = 100;

    recovL = left;
    recovR = right;
    pidLastError = error;
    Motor_SetSpeeds(left, right);
}
