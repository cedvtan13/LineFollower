/*
 * pid_controller.h  —  PID control logic for line following
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include "menu.h"  /* typedef PIDConfig and extern for pid */

/* Public state (read by UI and save/load) */
extern PIDConfig pid;

/* Initialize PID controller state */
void PID_Init(void);

/* Update PID: read sensors, compute output, set motor speeds */
void PID_Update(void);

/* Reset integral and derivative state (e.g., on stop) */
void PID_Reset(void);

#endif
