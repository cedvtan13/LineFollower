/*
 * pid_controller.h  —  simple PID line follower controller
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

void PID_Init(void);
void PID_Update(void);
void PID_Reset(void);

extern uint32_t runStartMs;
extern uint32_t runElapsedMs;
extern uint8_t  pidLineLost;

#endif
