/*
 * calibration.h  —  auto calibration sweep
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

#define CAL_IDLE 0u
#define CAL_SPIN 1u

extern uint8_t calState;

void Calibration_Init(void);
void Calibration_Start(void);
void Calibration_Update(void);
void Calibration_Abort(void);
uint8_t Calibration_IsDone(void);
uint32_t Calibration_TimeRemaining(void);

#endif