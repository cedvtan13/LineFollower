/*
 * pid_controller.c  —  PID control logic for line following
 *
 * Track-specific tuning notes (1-inch black line):
 *
 *  DASHED RECTANGLE   — line gaps at speed require a longer coast window.
 *                        CUT_COAST_MS=110 bridges a ~25 mm gap at 30 % speed.
 *
 *  ZIGZAG W-PEAKS     — abrupt direction reversals produce a large |dError|.
 *                        DERIV_BRAKE_DROP applies an extra quadratic slowdown
 *                        whenever the filtered derivative is large, in addition
 *                        to the positional corner scaling.
 *
 *  WAVE SEGMENTS      — small-amplitude sinusoidal curves oscillate the error
 *                        at high frequency.  A lower DERIV_ALPHA (more LP
 *                        filtering) prevents the derivative term from amplifying
 *                        the ripple.
 *
 *  SHARP CORNERS 45-90°  — CORNER_DROP raised, CORNER_MIN lowered so the robot
 *                          decelerates harder before tight angles.
 *
 *  LONG STRAIGHTS     — STRAIGHT_THR widened slightly; at least 10 straight
 *                        segments benefit from the full speed boost.
 */

#include "pid_controller.h"
#include "sensor.h"
#include "motor.h"
#include "stm32f4xx_hal.h"
#include <math.h>

/* ================================================================
   CONSTANTS
   ================================================================ */

/* Derivative low-pass filter.
 * Lower  = smoother (good for wave sections, reduces ripple amplification).
 * 0.28 vs previous 0.35. */
#define DERIV_ALPHA       0.28f

/* Positional corner speed scaling (quadratic drop on |errNorm|).
 * Raised from 0.65→0.72 for 8-10 sharp 45-90° corners. */
#define CORNER_DROP       0.72f
#define CORNER_MIN        0.22f   /* was 0.30 — allow deeper brake in tight spots */

/* Extra speed penalty when |derivative| is large (zigzag W-peaks).
 * Applied multiplicatively on top of positional corner scale.
 * At derivNorm=1 → speed multiplied by (1 - DERIV_BRAKE_DROP) = 0.40. */
#define DERIV_BRAKE_DROP  0.60f
#define DERIV_BRAKE_MIN   0.35f

/* Straight-line boost.
 * Threshold widened to 0.15 (was 0.12): more of the 10+ straight
 * segments qualify and trigger the full boost. */
#define STRAIGHT_THR      0.15f
#define STRAIGHT_BOOST    1.20f

/* Lost-line / dashed-line recovery timings.
 * COAST extended to 110 ms to bridge the ~25 mm dashed-rectangle gaps
 * (1-inch line ≈ 25 mm; at ≥30 % speed, gap crossing takes ~80-100 ms).
 * SEARCH reduced to 500 ms — on this track the line is always close. */
#define CUT_COAST_MS     110u   /* was 60  */
#define CUT_SEARCH_MS    500u   /* was 600 */
#define CUT_SEARCH_SPD    22    /* was 18  — slightly faster pivot search */

/* ================================================================
   PRIVATE STATE
   ================================================================ */

static float   pidIntegral  = 0.0f;
static float   pidLastError = 0.0f;
static float   pidDeriv     = 0.0f;

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
    PID_Reset();
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
       LOST-LINE / DASHED-LINE HANDLING
       ================================================================ */
    if (sensorActiveCount == 0) {
        if (lineLostAt == 0) lineLostAt = now;
        uint32_t lostMs = now - lineLostAt;

        if (lostMs < CUT_COAST_MS) {
            /* Phase 1 — COAST: hold last command, bridge dash gaps */
            Motor_SetSpeeds(recovL, recovR);
        } else if (lostMs < CUT_SEARCH_MS) {
            /* Phase 2 — PIVOT toward last known side */
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

    lineLostAt = 0;

    float error   = (float)linePosition;
    float errNorm = fabsf(error) / (float)SENSOR_POS_MAX;

    /* ================================================================
       INTEGRAL TERM
       ================================================================ */
    /* Reset on sign change to prevent windup through the many corners */
    if ((error > 0.0f) != (pidLastError > 0.0f))
        pidIntegral = 0.0f;

    pidIntegral += error;
    const float INT_CLAMP = (float)SENSOR_POS_MAX * 8.0f;
    if (pidIntegral >  INT_CLAMP) pidIntegral =  INT_CLAMP;
    if (pidIntegral < -INT_CLAMP) pidIntegral = -INT_CLAMP;

    /* ================================================================
       DERIVATIVE TERM  (low-pass filtered — 0.28 damps wave ripple)
       ================================================================ */
    float rawDeriv = error - pidLastError;
    pidDeriv       = DERIV_ALPHA * rawDeriv + (1.0f - DERIV_ALPHA) * pidDeriv;
    pidLastError   = error;

    /* ================================================================
       COMBINED PID OUTPUT
       ================================================================ */
    float output = pid.Kp * error
                 + pid.Ki * pidIntegral
                 + pid.Kd * pidDeriv;

    static const float STEER_SCALE = 100.0f / (float)SENSOR_POS_MAX;
    float steering = output * STEER_SCALE;

    /* ================================================================
       SPEED MANAGEMENT
       ================================================================ */

    /* 1. Positional corner slowdown (quadratic on |errNorm|).
     *    Handles large curves and sharp 45-90° corners. */
    float cornerScale = 1.0f - CORNER_DROP * (errNorm * errNorm);
    if (cornerScale < CORNER_MIN) cornerScale = CORNER_MIN;

    /* 2. Derivative-based brake for zigzag W-peaks.
     *    When the error is changing rapidly (large |dError/dt|), apply an
     *    additional quadratic slowdown independent of current position.
     *    derivNorm = |filtered_deriv| / SENSOR_POS_MAX, capped at 1. */
    float derivNorm  = fabsf(pidDeriv) / (float)SENSOR_POS_MAX;
    if (derivNorm > 1.0f) derivNorm = 1.0f;
    float derivScale = 1.0f - DERIV_BRAKE_DROP * (derivNorm * derivNorm);
    if (derivScale < DERIV_BRAKE_MIN) derivScale = DERIV_BRAKE_MIN;

    /* 3. Straight-line boost (wider threshold — benefits 10+ straight segments) */
    float boost = 1.0f;
    if (errNorm < STRAIGHT_THR) {
        float t = 1.0f - (errNorm / STRAIGHT_THR);
        boost = 1.0f + (STRAIGHT_BOOST - 1.0f) * t;
    }

    float effectiveBase = (float)pid.baseSpeed * cornerScale * derivScale * boost;
    if (effectiveBase > 100.0f) effectiveBase = 100.0f;

    int16_t L = (int16_t)(effectiveBase + steering);
    int16_t R = (int16_t)(effectiveBase - steering);

    if (L < -100) L = -100;
    if (L >  100) L =  100;
    if (R < -100) R = -100;
    if (R >  100) R =  100;

    recovL = L;
    recovR = R;

    Motor_SetSpeeds(L, R);
}
