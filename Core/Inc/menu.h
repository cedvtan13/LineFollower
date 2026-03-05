/*
 * menu.h  —  Application state machine: UI, PID, button handling
 *
 *  Created on: Feb 28, 2026
 *      Author: cvt
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#include "main.h"
#include "ssd1306.h"
#include "sensor.h"     /* sensorVal[], sensorRaw[], linePosition, sensorThreshold */
#include <stdint.h>

/* ----------------------------------------------------------------
   Application screens (flat state machine)
   ---------------------------------------------------------------- */
typedef enum {
    SCR_MAIN = 0,
    SCR_RUNNING,
    SCR_CALIBRATE,
    SCR_PID,
    SCR_SENSOR_DEBUG,
} AppScreen;

/* ----------------------------------------------------------------
   PID configuration (tunable from the PID screen)
   ---------------------------------------------------------------- */
typedef struct {
    float   Kp;
    float   Ki;
    float   Kd;
    uint8_t baseSpeed;   /* 0 – 100 % */
} PIDConfig;

/* ----------------------------------------------------------------
   Public globals — all defined in menu.c
   ---------------------------------------------------------------- */
extern PIDConfig pid;
extern AppScreen currentScreen;
extern uint8_t   calibrated;

/* UI cursor state */
extern uint8_t   mainCursor;  /* 0–3: main menu page index            */
extern uint8_t   pidCursor;   /* 0–4: PID field cursor (4 = SAVE)     */
extern uint8_t   pidEdit;     /* 1 while editing a PID value          */

/* Calibration state */
extern uint8_t   calState;    /* CAL_IDLE / CAL_SWEEP                 */

/* ----------------------------------------------------------------
   Public API — call from main.c
   ---------------------------------------------------------------- */
void App_Init(void);    /* call once after all peripheral inits */
void App_Update(void);  /* call every iteration of while(1)    */

#endif /* INC_MENU_H_ */
