/*
 * sensor.c  —  CD74HC4067SM 16-channel MUX + TCRT5000 IR sensor array
 *
 * POSITION CALCULATION  (updated for non-uniform geometry)
 * ----------------------------------------------------------
 * Physical layout (I0 = rightmost):
 *   I0,  I1       — curved to the right  (extra reach vs straight pitch)
 *   I2  … I13   — straight centre,  8 mm pitch  (1000 units/step)
 *   I14, I15     — curved to the left   (extra reach vs straight pitch)
 *
 * SENSOR_CURVE_EXTRA extra units are added per curved step so the
 * position table reflects the real physical detection point rather
 * than treating the array as uniformly spaced.
 *
 * TWO MODES
 * ---------
 * After auto-calibration (sensorCalibrated = 1):
 *   Analog-weighted average — each sensor contributes a continuous
 *   0.0–1.0 intensity (normalised by its own calMin/calMax).  Gives
 *   sub-sensor resolution and eliminates the 1000-unit stepping that
 *   binary thresholding produces.  PID works on smooth, continuous data.
 *
 * Without calibration:
 *   Binary-weighted average using the same physical position table.
 *   Better than the old uniform-step formula but still 1000-unit steps.
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
    /* Dummy read 1 */
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 5);
    HAL_ADC_GetValue(&hadc1);

    /* Dummy read 2 (optional, but helps) */
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 5);
    HAL_ADC_GetValue(&hadc1);

    /* Real read */
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) return 0;
    return (uint16_t)HAL_ADC_GetValue(&hadc1);
}

void Sensor_ReadAll(void)
{
    uint16_t mn = 4095u, mx = 0u;

    for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
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

    /* Compute sensorVal[] using the same threshold logic as
     * Sensor_ComputePosition so the debug screen always matches
     * what the PID actually sees. */
    uint16_t liveThr = (!sensorCalibrated && (mx - mn) > 200u)
                       ? (mn + mx) / 2u
                       : sensorThreshold;

    for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
        uint16_t thr = sensorCalibrated ? calThr[ch] : liveThr;
        sensorVal[ch] = (sensorFiltered[ch] > thr) ? 1u : 0u;
    }
}

int16_t Sensor_ComputePosition(void)
{
    if (sensorCalibrated) {
        /*
         * ANALOG MODE (post-calibration)
         * --------------------------------
         * Each sensor contributes a continuous 0.0–1.0 intensity:
         *   0.0 = pure white (at calMin floor)
         *   1.0 = black line (at calMax ceiling)
         * A 5% dead-zone at the bottom discards noise on fully white sensors.
         * This yields sub-sensor resolution and smooth PID input.
         */
        float weighted = 0.0f, total = 0.0f;
        uint8_t active = 0;
        for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
            uint16_t span = sensorCalMax[ch] - sensorCalMin[ch];
            if (span < CAL_MIN_SWING) continue;  /* skip uncalibrated sensor  */
            float norm = (float)((int16_t)sensorFiltered[ch] - (int16_t)sensorCalMin[ch])
                         / (float)span;
            if (norm < 0.25f) norm = 0.0f;   /* dead-zone: 25% — sensor must be clearly
                                               * above white baseline before it counts.
                                               * Rejects IR bleed onto adjacent white sensors
                                               * at 1 mm height with QRE1113. */
            else              active++;
            if (norm > 1.0f)  norm = 1.0f;
            weighted += norm * (float)SENS_POS[ch];
            total    += norm;
        }
        if (total < 0.15f) {
            /* Too little signal — line not seen (all white or lifted off surface) */
            sensorActiveCount = 0;
            return (linePosition >= 0) ? SENSOR_POS_MAX : -SENSOR_POS_MAX;
        }
        if (active >= 6) {
            /* More than 5 sensors active — a 1-inch line at 8 mm pitch physically
             * covers at most 3-4 sensors.  6+ firing means saturation or ambient
             * IR flood, not a real line. */
            sensorActiveCount = 0;
            return (linePosition >= 0) ? SENSOR_POS_MAX : -SENSOR_POS_MAX;
        }
        sensorActiveCount = active;
        linePosition = (int16_t)(weighted / total);
        return linePosition;
    } else {
        /*
         * CONFIDENCE-WEIGHTED MODE (before calibration)
         * -----------------------------------------------
         * Instead of a fixed 2048 threshold, compute a per-frame midpoint
         * liveThr = (min + max) / 2.  Sensors above the threshold contribute
         * a confidence weight proportional to how far above the threshold
         * they are.  This gives sub-sensor resolution even without cal data
         * (inspired by the Teensy maze-solver's confidence scoring).
         */
        uint16_t mn = 4095u, mx = 0u;
        for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
            if (sensorFiltered[ch] < mn) mn = sensorFiltered[ch];
            if (sensorFiltered[ch] > mx) mx = sensorFiltered[ch];
        }
        uint16_t span = mx - mn;
        uint16_t liveThr = (span > 200u) ? (mn + mx) / 2u : SENSOR_THR_DEF;

        float weighted = 0.0f, total = 0.0f;
        uint8_t active = 0;
        for (uint8_t ch = 0; ch < SENSOR_COUNT; ch++) {
            if (sensorFiltered[ch] <= liveThr) continue;
            /* Confidence = how far above threshold, normalised 0..1 */
            float conf = (span > 200u)
                       ? (float)(sensorFiltered[ch] - liveThr) / (float)(mx - liveThr)
                       : 1.0f;  /* fallback: equal weight */
            weighted += conf * (float)SENS_POS[ch];
            total    += conf;
            active++;
        }
        if (total < 0.01f) {
            sensorActiveCount = 0;
            return (linePosition >= 0) ? SENSOR_POS_MAX : -SENSOR_POS_MAX;
        }
        if (active >= 6) {
            sensorActiveCount = 0;
            return (linePosition >= 0) ? SENSOR_POS_MAX : -SENSOR_POS_MAX;
        }
        sensorActiveCount = active;
        linePosition = (int16_t)(weighted / total);
        return linePosition;
    }
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
        /* Always use the observed midpoint — never fall back to the fixed global
         * threshold (2048).  The global value is wrong at any non-zero height.
         * Even a narrow swing gives a better per-channel threshold than a
         * hardcoded constant that may sit inside the white-surface range. */
        calThr[ch] = (sensorCalMin[ch] + sensorCalMax[ch]) / 2u;
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
