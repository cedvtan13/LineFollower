/*
 * calibration.h  —  Auto-calibration for sensor thresholds
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>
#include "menu.h"  /* calState, calibrated declared in menu.h */

/* Calibration state values */
#define CAL_IDLE  0   /* idle — waiting for E press       */
#define CAL_SPIN  1   /* spinning clockwise, recording min/max */

/* calState and calibrated are declared in menu.h */

/* Initialize calibration state */
void Calibration_Init(void);

/* Start the auto-spin sweep */
void Calibration_Start(void);

/* Update calibration (called every loop — auto-finishes after CAL_SPIN_MS) */
void Calibration_Update(void);

/* Force-finish early (e.g. user aborts) */
void Calibration_Abort(void);

/*
 * Returns 1 (once) when the spin has finished and thresholds have been
 * computed.  Caller (App_Update) should then set calibrated=1 and switch
 * screen.  Clears itself on read.
 */
uint8_t Calibration_IsDone(void);

/* Milliseconds remaining in the current spin (0 when not spinning) */
uint32_t Calibration_TimeRemaining(void);

#endif
