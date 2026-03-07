/*
 * sensor.c  —  CD74HC4067SM 16-channel MUX + TCRT5000 IR sensor array
 *
 * Physical layout (I0 = rightmost):
 *   I0,  I1       — curved to the right  (extra reach vs straight pitch)
 *   I2  … I13   — straight centre,  8 mm pitch  (1000 units/step)
 *   I14, I15     — curved to the left   (extra reach vs straight pitch)
 *
 * POSITION CALCULATION
 * ---------------------
 * Binary weighted average: sensors above their threshold (per-channel after
 * calibration, SENSOR_THR_DEF before) are treated as ON.  The position is
 * the mean of the physical position values of all active sensors.
 */

#include "sensor.h"
#include <string.h>   /* memset */

extern ADC_HandleTypeDef hadc1;

uint8_t  sensorVal[SENSOR_COUNT]      = {0};
uint16_t sensorRaw[SENSOR_COUNT]      = {0};
uint16_t sensorFiltered[SENSOR_COUNT] = {0};  /* EMA-smoothed values */
int16_t  linePosition                 = 0;
uint16_t sensorThreshold          = SENSOR_THR_DEF;
uint8_t  sensorActiveCount        = 0;   /* 0 = line lost */

/* Auto-calibration data */
uint16_t sensorCalMin[SENSOR_COUNT];   /* lowest  ADC seen per channel (white) */
uint16_t sensorCalMax[SENSOR_COUNT];   /* highest ADC seen per channel (black) */
uint8_t  sensorCalibrated = 0;         /* 1 once Sensor_CalFinish() succeeds   */

/* Per-channel computed threshold (midpoint).  Valid only when sensorCalibrated=1. */
static uint16_t calThr[SENSOR_COUNT];

/*
 * Physical position lookup table.
 *   Units: 1000 = 8 mm = one straight sensor pitch.
 *   I0 (ch=0) is rightmost (+), I15 (ch=15) is leftmost (-).
 *
 * Straight section (I2–I13): uniform 1000-unit steps.
 * Curved sections (I0,I1 right / I14,I15 left): each step gains
 * SENSOR_CURVE_EXTRA units of extra reach.
 *
 *   Curved step 2 (I0/I15)   : ±(7500 + 2×CURVE_EXTRA) = ±8500
 *   Curved step 1 (I1/I14)   : ±(6500 + 1×CURVE_EXTRA) = ±7000
 *   Straight edge (I2/I13)   : ±5500
 *   Centre pair  (I7/I8)     : ± 500
 */
static const int16_t SENS_POS[SENSOR_COUNT] = {
  /*  ch   sensor   position */
    (int16_t)( 7500 + 2*SENSOR_CURVE_EXTRA),  /* I0  curved-right step 2 */
    (int16_t)( 6500 + 1*SENSOR_CURVE_EXTRA),  /* I1  curved-right step 1 */
     5500,   /* I2  */
     4500,   /* I3  */
     3500,   /* I4  */
     2500,   /* I5  */
     1500,   /* I6  */
      500,   /* I7  */
     -500,   /* I8  */
    -1500,   /* I9  */
    -2500,   /* I10 */
    -3500,   /* I11 */
    -4500,   /* I12 */
    -5500,   /* I13 */
    (int16_t)(-6500 - 1*SENSOR_CURVE_EXTRA),  /* I14 curved-left  step 1 */
    (int16_t)(-7500 - 2*SENSOR_CURVE_EXTRA),  /* I15 curved-left  step 2 */
};

/* Minimum ADC swing per sensor to be considered well-calibrated.
 * At 1 mm height contrast is strong — 600 ADC swing is easily achieved. */
#define CAL_MIN_SWING  600u

/*
 * MUX_Select: apply 4-bit channel address to S0-S3.
 * Followed by a short busy-wait for the CD74HC4067 to settle
 * (~400 ns max propagation delay at 3.3 V, 10 pF load).
 */
static void MUX_Select(uint8_t ch)
{
    HAL_GPIO_WritePin(MUX_S0_GPIO_Port, MUX_S0_Pin, (ch >> 0) & 1);
    HAL_GPIO_WritePin(MUX_S1_GPIO_Port, MUX_S1_Pin, (ch >> 1) & 1);
    HAL_GPIO_WritePin(MUX_S2_GPIO_Port, MUX_S2_Pin, (ch >> 2) & 1);
    HAL_GPIO_WritePin(MUX_S3_GPIO_Port, MUX_S3_Pin, (ch >> 3) & 1);
    /* 10 µs delay @ 96 MHz = 960 cycles */
    for (volatile int t = 0; t < 960; t++) { __NOP(); }
}

/*
 * ADC_ReadPB1: start a single conversion on ADC1_CH9 (PB1) and return result.
 * The ADC is already initialised and configured for channel 9 in main.c.
 */
static uint16_t ADC_ReadPB1(void)
{
    /* 4 dummy reads before the real one.
     * TCRT5000/QRE1113 output impedance can reach 1 MΩ at low light levels.
     * τ = 1 MΩ × 4 pF (S/H cap) = 4 µs → 8τ = 32 µs needed for 12-bit settling.
     * Each conversion at 480 ADC cycles @ 24 MHz = ~20.5 µs.
     * 4 dummies = ~82 µs total settle budget — covers up to ~2 MΩ source. */
    HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1, 5); HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1, 5); HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1, 5); HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Start(&hadc1); HAL_ADC_PollForConversion(&hadc1, 5); HAL_ADC_GetValue(&hadc1);

    /* Real read */
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) return 0;
    return (uint16_t)HAL_ADC_GetValue(&hadc1);
}

void Sensor_ReadAll(void)
{
    uint16_t mn = 4095u, mx = 0u;

    for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
        /* Channels 0 and 15 are the outermost curved sensors.
         * Ch 0 is physically broken; ch 15 is excluded for symmetry.
         * Zero their state so they never contribute to position. */
        if (ch == 0 || ch == 15) {
            sensorRaw[ch]      = 0u;
            sensorFiltered[ch] = 0u;
            sensorVal[ch]      = 0u;
            continue;
        }

        MUX_Select(ch);
        sensorRaw[ch] = ADC_ReadPB1();

        /* Exponential moving average — smooths per-read noise from the
         * high-impedance QRE1113 output.  Alpha=0.3 balances response speed
         * vs. noise rejection.  On first call (filtered==0), seed with raw. */
        if (sensorFiltered[ch] == 0)
            sensorFiltered[ch] = sensorRaw[ch];
        else
            sensorFiltered[ch] = (uint16_t)(0.3f * (float)sensorRaw[ch]
                                          + 0.7f * (float)sensorFiltered[ch]);

        if (sensorFiltered[ch] < mn) mn = sensorFiltered[ch];
        if (sensorFiltered[ch] > mx) mx = sensorFiltered[ch];
    }

    /* Threshold: use per-channel cal midpoint when calibrated, otherwise
     * SENSOR_THR_DEF unless a real line is visible (span >= 1000 means black
     * is present; white-only variation is only ~300 counts). */
    uint16_t liveThr = (!sensorCalibrated && (mx - mn) >= 1000u)
                       ? (mn + mx) / 2u
                       : sensorThreshold;

    for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
        if (sensorCalibrated
            && (sensorCalMax[ch] - sensorCalMin[ch]) < CAL_MIN_SWING) {
            sensorVal[ch] = 0u;  /* low-swing sensor — never report on-line */
        } else {
            uint16_t thr = sensorCalibrated ? calThr[ch] : liveThr;
            sensorVal[ch] = (sensorFiltered[ch] > thr) ? 1u : 0u;
        }
    }
}

int16_t Sensor_ComputePosition(void)
{
    /* Binary weighted average — use sensorVal[] already computed by Sensor_ReadAll(). */
    int32_t weighted = 0;
    uint8_t active   = 0;

    for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
        if (sensorVal[ch]) {
            if (ch == 0 || ch == 15) continue;  /* excluded channels */
            weighted += SENS_POS[ch];
            active++;
        }
    }

    if (active == 0) {
        sensorActiveCount = 0;
        return (linePosition >= 0) ? SENSOR_POS_MAX : -SENSOR_POS_MAX;
    }
    if (active >= 10) {
        /* 10+ sensors on — IR flood or robot lifted off surface. */
        sensorActiveCount = 0;
        return (linePosition >= 0) ? SENSOR_POS_MAX : -SENSOR_POS_MAX;
    }

    sensorActiveCount = active;
    linePosition = (int16_t)(weighted / (int32_t)active);
    return linePosition;
}

/* ==========================================================================
   AUTO-CALIBRATION
   ========================================================================== */

/* (CAL_MIN_SWING is defined above, near the position table) */

void Sensor_CalStart(void)
{
    for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
        sensorCalMin[ch] = 4095u;
        sensorCalMax[ch] = 0u;
    }
    sensorCalibrated = 0;
}

void Sensor_CalUpdate(void)
{
    for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
        if (ch == 0 || ch == 15) {
            sensorRaw[ch]      = 0u;
            sensorFiltered[ch] = 0u;
            sensorVal[ch]      = 0u;
            continue;
        }
        MUX_Select(ch);
        uint16_t raw = ADC_ReadPB1();
        sensorRaw[ch] = raw;

        /* Keep EMA updated during spin so the calibration screen
         * sensor bar shows filtered (stable) values. */
        if (sensorFiltered[ch] == 0)
            sensorFiltered[ch] = raw;
        else
            sensorFiltered[ch] = (uint16_t)(0.3f * (float)raw
                                          + 0.7f * (float)sensorFiltered[ch]);

        if (raw < sensorCalMin[ch]) sensorCalMin[ch] = raw;
        if (raw > sensorCalMax[ch]) sensorCalMax[ch] = raw;
        /* Live binary value using current cal data mid-point */
        uint16_t thr = (sensorCalMin[ch] + sensorCalMax[ch]) / 2u;
        sensorVal[ch] = (sensorFiltered[ch] > thr) ? 1u : 0u;
    }
}

void Sensor_CalFinish(void)
{
    for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
        /* Threshold at 65% of span rather than midpoint (50%).
         * Tarpaulin white surfaces have variable reflectivity — glossy patches
         * can read 1500+ vs normal 700-1000.  A higher threshold demands a
         * reading be 65% of the way from white to black before it counts,
         * preventing false on-line detections from bright white noise.
         *
         * Example: min=700, max=4000, span=3300
         *   Old midpoint: 2350  → noisy white at 1500 is 48%, close to trigger
         *   New 65%:      2845  → noisy white needs to reach 2845, safe margin */
        uint16_t span = sensorCalMax[ch] - sensorCalMin[ch];
        calThr[ch] = sensorCalMin[ch] + (uint16_t)((uint32_t)span * 65u / 100u);
    }
    sensorCalibrated = 1;

    /* Reset EMA filter — stale values from the calibration spin cause a
     * false all-black reading on the first Sensor_ComputePosition() call,
     * which makes the robot turn CW then stop immediately after cal. */
    memset(sensorFiltered, 0, sizeof(sensorFiltered));

    /* Seed with 3 fresh reads for a clean post-cal baseline */
    for (uint8_t flush = 0; flush < 3; flush++) {
        for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
            MUX_Select(ch);
            uint16_t raw = ADC_ReadPB1();
            sensorRaw[ch] = raw;
            if (sensorFiltered[ch] == 0)
                sensorFiltered[ch] = raw;
            else
                sensorFiltered[ch] = (uint16_t)(0.3f * (float)raw
                                              + 0.7f * (float)sensorFiltered[ch]);
        }
    }
}

uint8_t Sensor_CalConfidence(void)
{
    uint8_t count = 0;
    for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
        if ((sensorCalMax[ch] - sensorCalMin[ch]) >= CAL_MIN_SWING) count++;
    }
    return count;
}
