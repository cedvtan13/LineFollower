/*
 * menu.c  —  simple menu/state machine for run, PID, calibration, and debug
 */

#include "menu.h"
#include "calibration.h"
#include "input.h"
#include "motor.h"
#include "pid_controller.h"
#include "sensor.h"
#include "ui.h"
#include "stm32f4xx_hal.h"

PIDConfig pid = {0.55f, 0.18f, 28u, 16u};
AppScreen currentScreen = SCR_MAIN;
uint8_t calibrated = 0;
uint8_t mainCursor = 0;
uint8_t pidCursor = 0;
uint8_t pidEdit = 0;
uint8_t calState = CAL_IDLE;

static void Handle_Main(ButtonEvent ev)
{
    switch (ev) {
        case BTN_LEFT:
            mainCursor = (mainCursor > 0u) ? (uint8_t)(mainCursor - 1u) : 3u;
            UI_Refresh();
            break;
        case BTN_RIGHT:
            mainCursor = (mainCursor < 3u) ? (uint8_t)(mainCursor + 1u) : 0u;
            UI_Refresh();
            break;
        case BTN_ENTER_SHORT:
            if (mainCursor == 0u) {
                PID_Reset();
                runStartMs = HAL_GetTick();
                Motor_Enable();
                currentScreen = SCR_RUNNING;
            } else if (mainCursor == 1u) {
                currentScreen = SCR_PID;
                pidCursor = 0u;
                pidEdit = 0u;
            } else if (mainCursor == 2u) {
                currentScreen = SCR_CALIBRATE;
            } else {
                currentScreen = SCR_SENSOR_DEBUG;
            }
            UI_Refresh();
            break;
        default:
            break;
    }
}

static void Handle_Running(ButtonEvent ev)
{
    if (ev == BTN_ENTER_SHORT || ev == BTN_ENTER_LONG) {
        Motor_Stop();
        PID_Reset();
        currentScreen = SCR_MAIN;
        UI_Refresh();
    }
}

static void Handle_PID(ButtonEvent ev)
{
    if (ev == BTN_ENTER_LONG) {
        pidEdit = 0u;
        currentScreen = SCR_MAIN;
        UI_Refresh();
        return;
    }

    if (ev == BTN_ENTER_SHORT) {
        pidEdit = (uint8_t)!pidEdit;
        UI_Refresh();
        return;
    }

    if (!pidEdit) {
        if (ev == BTN_LEFT) {
            pidCursor = (pidCursor > 0u) ? (uint8_t)(pidCursor - 1u) : 3u;
            UI_Refresh();
        } else if (ev == BTN_RIGHT) {
            pidCursor = (pidCursor < 3u) ? (uint8_t)(pidCursor + 1u) : 0u;
            UI_Refresh();
        }
        return;
    }

    if (ev == BTN_LEFT) {
        switch (pidCursor) {
            case 0: pid.Kp -= 0.05f; if (pid.Kp < 0.0f) pid.Kp = 0.0f; break;
            case 1: pid.Kd -= 0.05f; if (pid.Kd < 0.0f) pid.Kd = 0.0f; break;
            case 2: if (pid.baseSpeed >= 5u) pid.baseSpeed -= 5u; break;
            case 3: if (pid.turnSpeed >= 5u) pid.turnSpeed -= 5u; break;
            default: break;
        }
        UI_Refresh();
    } else if (ev == BTN_RIGHT) {
        switch (pidCursor) {
            case 0: pid.Kp += 0.05f; if (pid.Kp > 10.0f) pid.Kp = 10.0f; break;
            case 1: pid.Kd += 0.05f; if (pid.Kd > 10.0f) pid.Kd = 10.0f; break;
            case 2: if (pid.baseSpeed <= 95u) pid.baseSpeed += 5u; break;
            case 3: if (pid.turnSpeed <= 95u) pid.turnSpeed += 5u; break;
            default: break;
        }
        UI_Refresh();
    }
}

static void Handle_Calibrate(ButtonEvent ev)
{
    if (ev == BTN_ENTER_SHORT) {
        if (calState == CAL_IDLE) Calibration_Start();
        else Calibration_Abort();
        UI_Refresh();
    } else if (ev == BTN_ENTER_LONG || ev == BTN_LEFT) {
        Calibration_Abort();
        currentScreen = SCR_MAIN;
        UI_Refresh();
    }
}

static void Handle_SensorDebug(ButtonEvent ev)
{
    if (ev == BTN_LEFT || ev == BTN_ENTER_SHORT || ev == BTN_ENTER_LONG) {
        currentScreen = SCR_MAIN;
        UI_Refresh();
    }
}

void App_Init(void)
{
    Input_Init();
    Calibration_Init();
    PID_Init();
    UI_Init();
}

void App_Update(void)
{
    static uint32_t lastUiMs = 0;
    uint32_t now = HAL_GetTick();
    ButtonEvent ev;

    Input_Update();
    ev = Input_GetEvent();

    switch (currentScreen) {
        case SCR_MAIN:         Handle_Main(ev); break;
        case SCR_RUNNING:      Handle_Running(ev); break;
        case SCR_PID:          Handle_PID(ev); break;
        case SCR_CALIBRATE:    Handle_Calibrate(ev); break;
        case SCR_SENSOR_DEBUG: Handle_SensorDebug(ev); break;
    }

    if (currentScreen == SCR_RUNNING) {
        PID_Update();
    } else if (currentScreen == SCR_SENSOR_DEBUG) {
        Sensor_ReadAll();
        linePosition = Sensor_ComputePosition();
    } else if (currentScreen == SCR_CALIBRATE) {
        Calibration_Update();
        if (Calibration_IsDone()) {
            calibrated = 1u;
            currentScreen = SCR_MAIN;
        }
    }

    if ((now - lastUiMs) >= 100u || ev != BTN_NONE) {
        lastUiMs = now;
        UI_Refresh();
    }
}
