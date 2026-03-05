/*
 * pid_controller.c  —  PID control logic for line following
 */

#include "pid_controller.h"
#include "sensor.h"
#include "motor.h"
#include "stm32f4xx_hal.h"
#include <math.h>

/* ================================================================
   CONSTANTS (tunable)
   ================================================================ */

/* Derivative low-pass  (0 = frozen  …  1 = raw) */
#define DERIV_ALPHA       0.35f

/* Corner speed scaling  (quadratic drop) */
#define CORNER_DROP       0.65f
#define CORNER_MIN        0.30f

/* Straight-line boost */
#define STRAIGHT_THR      0.12f
#define STRAIGHT_BOOST    1.20f

/* Cut-line recovery timings (milliseconds) */
#define CUT_COAST_MS      60u
#define CUT_SEARCH_MS    600u
#define CUT_SEARCH_SPD    18

/* ================================================================
   PUBLIC GLOBALS
   ================================================================ */

/* Uses pid from pid_controller.h which is extern from menu.h */

/* ================================================================
   PRIVATE STATE
   ================================================================ */

static float   pidIntegral  = 0.0f;
static float   pidLastError = 0.0f;
static float   pidDeriv     = 0.0f;

/* Cut-line / lost-line recovery */
static uint32_t lineLostAt = 0;
static int16_t  recovL     = 0;
static int16_t  recovR     = 0;

/* ================================================================
   INITIALIZATION
   ================================================================ */

void PID_Reset(void)
{
    pidIntegral  = 0.0f;
    pidLastError = 0.0f;
    pidDeriv     = 0.0f;
    lineLostAt   = 0;
    recovL       = 0;
    recovR       = 0;
}

void PID_Init(void)
{
    PID_Reset();   /* same state — single point of truth */
}

/* ================================================================
   MAIN UPDATE
   ================================================================ */

void PID_Update(void)
{
    Sensor_ReadAll();
    linePosition = Sensor_ComputePosition();
    uint32_t now = HAL_GetTick();

    /* ================================================================
       LOST-LINE HANDLING
       ================================================================ */
    if (sensorActiveCount == 0) {
        if (lineLostAt == 0) lineLostAt = now;
        uint32_t lostMs = now - lineLostAt;

        if (lostMs < CUT_COAST_MS) {
            /* Phase 1 — COAST THROUGH */
            Motor_SetSpeeds(recovL, recovR);
        } else if (lostMs < CUT_SEARCH_MS) {
            /* Phase 2 — PIVOT SEARCH */
            int16_t spd = CUT_SEARCH_SPD;
            if (linePosition >= 0)
                Motor_SetSpeeds( spd, -spd);
            else
                Motor_SetSpeeds(-spd,  spd);
        } else {
            /* Phase 3 — HARD BRAKE */
            Motor_HardBrake();
        }
        return;
    }

    /* Line is visible — reset lost timer */
    lineLostAt = 0;

    float error   = (float)linePosition;
    float errNorm = fabsf(error) / (float)SENSOR_POS_MAX;

    /* ================================================================
       INTEGRAL TERM
       ================================================================ */
    if ((error > 0.0f) != (pidLastError > 0.0f))
        pidIntegral = 0.0f;

    pidIntegral += error;
    const float INT_CLAMP = (float)(SENSOR_POS_MAX) * 8.0f;
    if (pidIntegral >  INT_CLAMP) pidIntegral =  INT_CLAMP;
    if (pidIntegral < -INT_CLAMP) pidIntegral = -INT_CLAMP;

    /* ================================================================
       DERIVATIVE TERM  (low-pass filtered)
       ================================================================ */
    float rawDeriv = error - pidLastError;
    pidDeriv       = DERIV_ALPHA * rawDeriv + (1.0f - DERIV_ALPHA) * pidDeriv;
    pidLastError   = error;

    /* ================================================================
       COMBINED OUTPUT
       ================================================================ */
    float output = pid.Kp * error
                 + pid.Ki * pidIntegral
                 + pid.Kd * pidDeriv;

    /* SENSOR_POS_MAX / 100 is a compile-time constant — multiply is faster than divide */
    static const float STEER_SCALE = 100.0f / (float)SENSOR_POS_MAX;
    float steering = output * STEER_SCALE;

    /* ================================================================
       SPEED MANAGEMENT
       ================================================================ */

    /* Corner slowdown (quadratic) */
    float cornerScale = 1.0f - CORNER_DROP * (errNorm * errNorm);
    if (cornerScale < CORNER_MIN) cornerScale = CORNER_MIN;

    /* Straight-line boost */
    float boost = 1.0f;
    if (errNorm < STRAIGHT_THR) {
        float t = 1.0f - (errNorm / STRAIGHT_THR);
        boost = 1.0f + (STRAIGHT_BOOST - 1.0f) * t;
    }

    float effectiveBase = (float)pid.baseSpeed * cornerScale * boost;
    if (effectiveBase > 100.0f) effectiveBase = 100.0f;

    int16_t L = (int16_t)(effectiveBase + steering);
    int16_t R = (int16_t)(effectiveBase - steering);

    if (L < -100) L = -100;
    if (L >  100) L =  100;
    if (R < -100) R = -100;
    if (R >  100) R =  100;

    /* Save for coast-through recovery */
    recovL = L;
    recovR = R;

    Motor_SetSpeeds(L, R);
}
