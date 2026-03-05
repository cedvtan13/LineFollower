/*
 * menu.c  —  Application state machine coordinator
 *
 *  Created on: Feb 28, 2026
 *      Author: cvt
 *
 *  Architecture: flat 4-screen state machine coordinating:
 *    – pid_controller.c   PID control and motor output
 *    – calibration.c      Auto-calibration sweep
 *    – ui.c               Display rendering
 *    – input.c            Button debouncing and events
 *
 *  Delegates to specialized modules instead of monolithic code.
 */

#include "menu.h"
#include "pid_controller.h"
#include "calibration.h"
#include "ui.h"
#include "input.h"
#include "sensor.h"
#include "motor.h"
#include "flash_storage.h"
#include "ssd1306.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* ================================================================
   PUBLIC GLOBALS (definitions — extern declared in menu.h)
   ================================================================ */
PIDConfig pid            = {2.0f, 0.0f, 3.0f, 30};
AppScreen currentScreen  = SCR_MAIN;
uint8_t   calibrated     = 0;

uint8_t   mainCursor     = 0;   /* main menu page 0–3                */
uint8_t   pidCursor      = 0;   /* PID cursor 0–4 (4 = SAVE)         */
uint8_t   pidEdit        = 0;   /* 1 while editing a PID value       */

uint8_t   calState       = CAL_IDLE;

/* ================================================================
   FORWARD DECLARATIONS
   ================================================================ */

static void Handle_MainMenu(ButtonEvent ev);
static void Handle_Running(ButtonEvent ev);
static void Handle_Calibrate(ButtonEvent ev);
static void Handle_PID(ButtonEvent ev);
static void Handle_SensorDebug(ButtonEvent ev);

/* ================================================================
   STATE MACHINE ROUTING
   ================================================================ */

static void Route_ButtonEvent(ButtonEvent ev)
{
    switch (currentScreen) {
        case SCR_MAIN:         Handle_MainMenu(ev);    break;
        case SCR_RUNNING:      Handle_Running(ev);     break;
        case SCR_CALIBRATE:    Handle_Calibrate(ev);   break;
        case SCR_PID:          Handle_PID(ev);         break;
        case SCR_SENSOR_DEBUG: Handle_SensorDebug(ev); break;
    }
}

/* ================================================================
   MAIN MENU STATE HANDLER
   ================================================================ */

static void Handle_MainMenu(ButtonEvent ev)
{
    switch (ev) {
        case BTN_LEFT:
            mainCursor = (mainCursor > 0) ? (mainCursor - 1) : 3;
            UI_Refresh();
            break;

        case BTN_RIGHT:
            mainCursor = (mainCursor < 3) ? (mainCursor + 1) : 0;
            UI_Refresh();
            break;

        case BTN_ENTER_SHORT:
            switch (mainCursor) {
                case 0: /* RUN */
                    if (!calibrated) {
                        ssd1306_Clear();
                        ssd1306_SetCursor(4, 3);
                        ssd1306_WriteString("Calibrate first!");
                        ssd1306_Display();
                        HAL_Delay(1500);
                        UI_Refresh();
                    } else {
                        PID_Reset();
                        Motor_Enable();
                        currentScreen = SCR_RUNNING;
                        UI_Refresh();
                    }
                    break;

                case 1: /* CALIBRATE — start spin directly, no idle screen */
                    Calibration_Start();
                    currentScreen = SCR_CALIBRATE;
                    UI_Refresh();
                    break;

                case 2: /* PID */
                    currentScreen = SCR_PID;
                    pidCursor = 0;
                    pidEdit = 0;
                    UI_Refresh();
                    break;

                case 3: /* SENSOR DEBUG */
                    currentScreen = SCR_SENSOR_DEBUG;
                    UI_Refresh();
                    break;
            }
            break;

        case BTN_ENTER_LONG:
        case BTN_NONE:
        default:
            break;
    }
}

/* ================================================================
   RUNNING STATE HANDLER
   ================================================================ */

static void Handle_Running(ButtonEvent ev)
{
    switch (ev) {
        case BTN_ENTER_SHORT:
        case BTN_ENTER_LONG:
            Motor_Stop();
            PID_Reset();
            currentScreen = SCR_MAIN;
            UI_Refresh();
            break;

        case BTN_LEFT:
        case BTN_RIGHT:
        case BTN_NONE:
        default:
            break;
    }
}

/* ================================================================
   CALIBRATION STATE HANDLER
   ================================================================ */

static void Handle_Calibrate(ButtonEvent ev)
{
    switch (ev) {
        case BTN_ENTER_SHORT:
            if (calState == CAL_IDLE) {
                /* Start the auto-spin sweep */
                Calibration_Start();
                UI_Refresh();
            } else {
                /* E during spin — abort immediately */
                Calibration_Abort();
                UI_Refresh();
            }
            break;

        case BTN_LEFT:
            if (calState == CAL_SPIN) {
                Calibration_Abort();
            } else {
                currentScreen = SCR_MAIN;
            }
            UI_Refresh();
            break;

        case BTN_ENTER_LONG:
            Calibration_Abort();
            PID_Reset();
            currentScreen = SCR_MAIN;
            UI_Refresh();
            break;

        case BTN_RIGHT:
        case BTN_NONE:
        default:
            break;
    }
}

/* ================================================================
   PID TUNING STATE HANDLER
   ================================================================ */

static void Handle_PID(ButtonEvent ev)
{
    switch (ev) {
        case BTN_LEFT:
            if (pidEdit) {
                switch (pidCursor) {
                    case 0: pid.Kp -= 0.1f;  if (pid.Kp < 0.0f) pid.Kp = 0.0f; break;
                    case 1: pid.Ki -= 0.01f; if (pid.Ki < 0.0f) pid.Ki = 0.0f; break;
                    case 2: pid.Kd -= 0.05f; if (pid.Kd < 0.0f) pid.Kd = 0.0f; break;
                    case 3: if (pid.baseSpeed >= 5) pid.baseSpeed -= 5; break;
                }
            } else {
                if (pidCursor > 0) pidCursor--;
            }
            UI_Refresh();
            break;

        case BTN_RIGHT:
            if (pidEdit) {
                switch (pidCursor) {
                    case 0: pid.Kp += 0.1f;  if (pid.Kp > 10.0f) pid.Kp = 10.0f; break;
                    case 1: pid.Ki += 0.01f; if (pid.Ki >  2.0f) pid.Ki =  2.0f; break;
                    case 2: pid.Kd += 0.05f; if (pid.Kd >  5.0f) pid.Kd =  5.0f; break;
                    case 3: if (pid.baseSpeed <= 95) pid.baseSpeed += 5; break;
                }
            } else {
                if (pidCursor < 4) pidCursor++;
            }
            UI_Refresh();
            break;

        case BTN_ENTER_SHORT:
            if (pidCursor == 4) {
                /* SAVE */
                uint8_t ok = FlashStorage_Save(&pid);
                ssd1306_Clear();
                ssd1306_SetCursor(16, 3);
                ssd1306_WriteString(ok ? "PID  SAVED!" : "SAVE FAILED");
                ssd1306_Display();
                HAL_Delay(1200);
                UI_Refresh();
            } else {
                pidEdit = !pidEdit;
                UI_Refresh();
            }
            break;

        case BTN_ENTER_LONG:
        case BTN_NONE:
        default:
            break;
    }
}

/* ================================================================
   SENSOR DEBUG STATE HANDLER
   ================================================================ */

static void Handle_SensorDebug(ButtonEvent ev)
{
    switch (ev) {
        case BTN_ENTER_SHORT:
        case BTN_LEFT:
            currentScreen = SCR_MAIN;
            UI_Refresh();
            break;

        case BTN_RIGHT:
        case BTN_ENTER_LONG:
        case BTN_NONE:
        default:
            break;
    }
}

/* ================================================================
   PUBLIC API
   ================================================================ */

void App_Init(void)
{
    /* Load PID from flash (if previously saved) */
    FlashStorage_Load(&pid);

    /* Initialize sub-modules */
    PID_Init();          /* zero integral/derivative state               */
    calState      = CAL_IDLE;
    calibrated    = 0;   /* cleared at boot; set only after first cal    */
    Input_Init();
    UI_Init();
}

/*
 * Call this every iteration of while(1) in main.c.
 * – Reads buttons and dispatches state handlers
 * – In SCR_RUNNING: runs PID every call, refreshes display every 150 ms
 * – In SCR_CALIBRATE: runs calibration update every call, refreshes display every 100 ms
 * – In SCR_SENSOR_DEBUG: reads sensors continuously, refreshes display every 100 ms
 */
void App_Update(void)
{
    /* Update button input */
    Input_Update();
    ButtonEvent ev = Input_GetEvent();

    /* Handle button events with state-specific logic */
    if (ev != BTN_NONE) {
        Route_ButtonEvent(ev);
    }

    /* Fast periodic updates per screen
     * Each screen has its own lastDisp so switching screens
     * always triggers an immediate refresh on arrival.
     */
    static uint32_t lastDispRun  = 0;
    static uint32_t lastDispDbg  = 0;
    static uint32_t lastDispCal  = 0;
    uint32_t now = HAL_GetTick();

    if (currentScreen == SCR_RUNNING) {
        /* Run PID controller every iteration (fast path) */
        PID_Update();

        /* Refresh display every 150 ms */
        if ((now - lastDispRun) >= 150u) {
            lastDispRun = now;
            UI_Refresh();
        }
    } else if (currentScreen == SCR_SENSOR_DEBUG) {
        /* Read all 16 sensors continuously */
        Sensor_ReadAll();

        /* Refresh display every 100 ms */
        if ((now - lastDispDbg) >= 100u) {
            lastDispDbg = now;
            UI_Refresh();
        }
    } else if (currentScreen == SCR_CALIBRATE) {
        /* Update calibration (reads sensors / drives motors during spin) */
        Calibration_Update();

        /* Auto-finish: motor already stopped inside Calibration_Update */
        if (Calibration_IsDone()) {
            calibrated    = 1;
            currentScreen = SCR_MAIN;
            UI_Refresh();
        } else if ((now - lastDispCal) >= 100u) {
            lastDispCal = now;
            UI_Refresh();
        }
    }
}
