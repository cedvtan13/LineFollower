/*
 * calibration.h  —  Auto-calibration for sensor thresholds
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>
#include "menu.h"  /* extern calibrated */

/* Calibration state */
#define CAL_IDLE  0   /* showing instructions, waiting for E */
#define CAL_SWEEP 1   /* actively sweeping, recording min/max */

extern uint8_t calState;
extern uint8_t calibrated;

/* Initialize calibration state */
void Calibration_Init(void);

/* Start a new calibration sweep */
void Calibration_Start(void);

/* Update calibration (called every loop during sweep) */
void Calibration_Update(void);

/* Finish calibration and compute thresholds */
void Calibration_Finish(void);

/* Abort current calibration */
void Calibration_Abort(void);

#endif
