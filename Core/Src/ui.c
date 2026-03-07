/*
 * ui.c  —  simple OLED rendering
 */

#include "ui.h"
#include "calibration.h"
#include "menu.h"
#include "pid_controller.h"
#include "sensor.h"
#include "sh1106.h"
#include <math.h>
#include <stdio.h>

static char buf[24];

static void DrawHeader(const char *title)
{
    sh1106_Clear();
    sh1106_SetCursor(0, 0);
    sh1106_WriteString(title);
    sh1106_FillRect(0, 8, 127, 8);
}

static void DrawSensorBar(uint8_t row)
{
    int8_t ch;
    sh1106_SetCursor(0, row);
    sh1106_WriteString("L ");
    for (ch = 15; ch >= 0; ch--) {
        char c[2] = {sensorVal[ch] ? '*' : '.', '\0'};
        sh1106_WriteString(c);
    }
}

static void UI_DrawMain(void)
{
    static const char *items[4] = {"RUN", "PID", "CAL", "SENS"};
    uint8_t i;

    DrawHeader("LINE FOLLOWER");
    snprintf(buf, sizeof(buf), "CAL:%s", calibrated ? "DONE" : "NO");
    sh1106_SetCursor(0, 2);
    sh1106_WriteString(buf);

    for (i = 0; i < 4u; i++) {
        sh1106_SetCursor(0, (uint8_t)(3u + i));
        sh1106_WriteString((i == mainCursor) ? "> " : "  ");
        sh1106_WriteString(items[i]);
    }
}

static void UI_DrawRunning(void)
{
    DrawHeader("RUNNING");
    DrawSensorBar(2);
    snprintf(buf, sizeof(buf), "POS:%6d ACT:%2u", linePosition, sensorActiveCount);
    sh1106_SetCursor(0, 4);
    sh1106_WriteString(buf);
    snprintf(buf, sizeof(buf), "T:%lu.%lus %s",
             (unsigned long)(runElapsedMs / 1000u),
             (unsigned long)((runElapsedMs % 1000u) / 100u),
             pidLineLost ? "LOST" : "OK");
    sh1106_SetCursor(0, 5);
    sh1106_WriteString(buf);
    sh1106_SetCursor(0, 7);
    sh1106_WriteString("E:STOP");
}

static void UI_DrawPID(void)
{
    int ip;
    int fp;

    DrawHeader("PID TUNING");
    ip = (int)pid.Kp;
    fp = (int)roundf(fabsf(pid.Kp - (float)ip) * 100.0f);
    snprintf(buf, sizeof(buf), "%cKp  %d.%02d", pidCursor == 0u ? (pidEdit ? '*' : '>') : ' ', ip, fp);
    sh1106_SetCursor(0, 2);
    sh1106_WriteString(buf);

    ip = (int)pid.Kd;
    fp = (int)roundf(fabsf(pid.Kd - (float)ip) * 100.0f);
    snprintf(buf, sizeof(buf), "%cKd  %d.%02d", pidCursor == 1u ? (pidEdit ? '*' : '>') : ' ', ip, fp);
    sh1106_SetCursor(0, 3);
    sh1106_WriteString(buf);

    snprintf(buf, sizeof(buf), "%cSpd %3u%%", pidCursor == 2u ? (pidEdit ? '*' : '>') : ' ', pid.baseSpeed);
    sh1106_SetCursor(0, 4);
    sh1106_WriteString(buf);

    snprintf(buf, sizeof(buf), "%cTrn %3u%%", pidCursor == 3u ? (pidEdit ? '*' : '>') : ' ', pid.turnSpeed);
    sh1106_SetCursor(0, 5);
    sh1106_WriteString(buf);

    sh1106_SetCursor(0, 7);
    sh1106_WriteString(pidEdit ? "L/R:chg E:done" : "L/R:mv E:edit");
}

static void UI_DrawCalibrate(void)
{
    if (calState == CAL_IDLE) {
        DrawHeader("CALIBRATE");
        sh1106_SetCursor(0, 2);
        sh1106_WriteString("Place on line");
        sh1106_SetCursor(0, 3);
        sh1106_WriteString("Press E to spin");
        sh1106_SetCursor(0, 5);
        sh1106_WriteString("L/hold E:back");
    } else {
        snprintf(buf, sizeof(buf), "CAL %lu.%lus",
                 (unsigned long)(Calibration_TimeRemaining() / 1000u),
                 (unsigned long)((Calibration_TimeRemaining() % 1000u) / 100u));
        DrawHeader(buf);
        DrawSensorBar(2);
        snprintf(buf, sizeof(buf), "READY:%2u/16", Sensor_CalConfidence());
        sh1106_SetCursor(0, 5);
        sh1106_WriteString(buf);
        sh1106_SetCursor(0, 7);
        sh1106_WriteString("E:abort");
    }
}

static void UI_DrawDebug(void)
{
    uint8_t i;

    DrawHeader("SENSOR DEBUG");
    for (i = 0; i < 5u; i++) {
        uint8_t a = (uint8_t)(1u + i);
        uint8_t b = (uint8_t)(8u + i);
        snprintf(buf, sizeof(buf), "%02u:%4u %02u:%4u", a, sensorRaw[a], b, sensorRaw[b]);
        sh1106_SetCursor(0, (uint8_t)(2u + i));
        sh1106_WriteString(buf);
    }
    sh1106_SetCursor(0, 7);
    sh1106_WriteString("L/E:back");
}

void UI_Init(void)
{
    sh1106_Init();
    UI_Refresh();
}

void UI_Refresh(void)
{
    switch (currentScreen) {
        case SCR_MAIN: UI_DrawMain(); break;
        case SCR_RUNNING: UI_DrawRunning(); break;
        case SCR_PID: UI_DrawPID(); break;
        case SCR_CALIBRATE: UI_DrawCalibrate(); break;
        case SCR_SENSOR_DEBUG: UI_DrawDebug(); break;
    }
    sh1106_Display();
}
