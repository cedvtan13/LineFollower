/*
 * pid_controller.h  —  PID control logic for line following
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include "menu.h"  /* PIDConfig typedef, pid, AppScreen */

/* Initialize PID controller state */
/* (pid is already declared extern in menu.h) */
void PID_Init(void);

/* Update PID: read sensors, compute output, set motor speeds */
void PID_Update(void);

/* Reset integral and derivative state (e.g., on stop) */
void PID_Reset(void);

/* Performance metrics (updated in PID_Update) */
extern uint32_t runStartMs;     /* set to HAL_GetTick() when run starts */
extern uint32_t runElapsedMs;   /* elapsed ms since run started         */

/* Health monitor */
extern uint8_t  pidOscillating; /* 1 = PID error oscillating too fast  */

#endif
