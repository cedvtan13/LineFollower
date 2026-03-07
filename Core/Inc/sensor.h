/*
 * sensor.h  —  CD74HC4067SM 16-channel MUX + TCRT5000 IR sensor array
 *
 * MUX wiring:
 *   S0 = PB0    S1 = PA7    S2 = PA6    S3 = PA5   (channel select, outputs)
 *   SIG = PB1   (common I/O → ADC1_CH9)
 *   I0  = rightmost sensor (positive position side, excluded)
 *   I15 = leftmost  sensor (negative position side, excluded)
 *
 * ADC: 12-bit, 0–4095.
 *   Black line: high IR absorption → low phototransistor current → HIGH voltage → HIGH ADC
 *   White surface: high reflection → HIGH current → LOW voltage → LOW ADC
 *   ON LINE when sensorRaw[i] > per-channel threshold (or global sensorThreshold if not yet cal'd)
 *
 * Active position range: -7000 (line hard-left/I14) … 0 (centred) … +7000 (hard-right/I1)
 *
 * AUTO-CALIBRATION:
 *   1. Call Sensor_CalStart()  — resets min/max accumulators
 *   2. Call Sensor_CalUpdate() in a tight loop while sweeping sensors over the line
 *   3. Call Sensor_CalFinish() — computes per-channel midpoint thresholds
 *   After Sensor_CalFinish(), sensorCalibrated=1 and Sensor_ReadAll() uses
 *   per-channel thresholds instead of the global sensorThreshold.
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"
#include <stdint.h>

#define SENSOR_COUNT   16
#define SENSOR_THR_DEF 2500   /* fallback threshold: midpoint of white(~1350) and black(~3600) */

/* -----------------------------------------------------------------------
 * SENSOR GEOMETRY — tune these two values to match your hardware
 * -----------------------------------------------------------------------
 * Your array has:
 *   I0, I1       — curved to the RIGHT  (rightmost 2)
 *   I2  … I13   — straight centre section (8 mm pitch)
 *   I14, I15     — curved to the LEFT   (leftmost 2)
 *
 * SENSOR_CURVE_EXTRA:
 *   Extra position units added per curved sensor step beyond the straight
 *   section.  Nominal straight pitch = 1000 units (= 8 mm).
 *   The curved sensors arc outward, so their effective detection point is
 *   further from centre than a straight extrapolation would give.
 *   Rule of thumb: measure the arc chord vs straight continuation:
 *     ~4 mm extra → 500 units   ~8 mm extra → 1000 units
 *   Start with 500.  If the robot cuts corners, increase it.
 * --------------------------------------------------------------------- */
#define SENSOR_CURVE_EXTRA  500   /* extra reach per curved step (units)   */

/* Maximum possible position returned by Sensor_ComputePosition().        */
/* = position of I0 (or I15) when only that outermost sensor is active.   */
#define SENSOR_POS_MAX  (6500 + 1 * SENSOR_CURVE_EXTRA)  /* = 7000 with I1/I14 outermost */

/* Runtime state ----------------------------------------------------------- */
extern uint8_t  sensorVal[SENSOR_COUNT];  /* 1 = on line, 0 = off line     */
extern uint16_t sensorRaw[SENSOR_COUNT];  /* raw 12-bit ADC per channel     */
extern uint16_t sensorFiltered[SENSOR_COUNT]; /* EMA-smoothed ADC per channel */
extern int16_t  linePosition;             /* -POS_MAX … 0 … +POS_MAX          */
extern uint16_t sensorThreshold;          /* global fallback threshold       */
extern uint8_t  sensorActiveCount;        /* sensors contributing to fix (0 = line lost) */

/* Auto-calibration data --------------------------------------------------- */
extern uint16_t sensorCalMin[SENSOR_COUNT]; /* per-channel min ADC (white)  */
extern uint16_t sensorCalMax[SENSOR_COUNT]; /* per-channel max ADC (black)  */
extern uint8_t  sensorCalibrated;           /* 1 = cal data valid            */

/* -------------------------------------------------------------------------
 * Sensor_ReadAll:
 *   Reads all 16 channels through the MUX.
 *   Channels 0 and 15 are ignored in this project.
 *   If sensorCalibrated, uses per-channel midpoint threshold.
 *   Otherwise uses the global sensorThreshold.
 */
void    Sensor_ReadAll(void);

/*
 * Compute weighted-average position from sensorVal[].
 * Writes result into linePosition and returns it.
 * LINE LOST: returns ±SENSOR_POS_MAX toward the side the line last disappeared on.
 */
int16_t Sensor_ComputePosition(void);

/* -------------------------------------------------------------------------
 * Auto-calibration API
 * -------------------------------------------------------------------------
 * Typical usage in while(1) during a sweep:
 *
 *   Sensor_CalStart();            // reset accumulators
 *   while (user_still_sweeping)
 *       Sensor_CalUpdate();       // records ongoing min/max
 *   Sensor_CalFinish();           // computes per-channel thresholds
 */
void Sensor_CalStart(void);    /* reset min/max accumulators              */
void Sensor_CalUpdate(void);   /* read + update min/max (call every loop) */
void Sensor_CalFinish(void);   /* set per-channel thresholds, set flag    */

/* Returns how many channels have seen enough ADC swing to be confident.   */
uint8_t Sensor_CalConfidence(void);

#endif /* INC_SENSOR_H_ */
