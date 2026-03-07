/*
 * ui.c  —  UI/display rendering
 *
 * Layout: row 0 = title, y=8 = 1px divider, rows 2-6 = content, row 7 = hint.
 */

#include "ui.h"
#include "sh1106.h"
#include "pid_controller.h"
#include "calibration.h"
#include "sensor.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static char buf[32];

static void UI_WriteChar(char c)
{
    char tmp[2] = {c, '\0'};
    sh1106_WriteString(tmp);
}

/* Title row + thin 1px divider. page=0 → no page number. */
static void DrawHeader(const char *title, uint8_t page)
{
    sh1106_SetCursor(0, 0);
    sh1106_WriteString(title);
    if (page) {
        char pg[8];
        sprintf(pg, "%d/3", page);
        sh1106_SetCursor(116, 0);
        sh1106_WriteString(pg);
    }
    sh1106_FillRect(0, 8, 127, 8); /* 1 px divider */
}

/* ================================================================
   SCREEN DRAWS
   ================================================================ */

static void UI_DrawMainMenu(void)
{
    sh1106_Clear();

    if (mainCursor == 0) {
        DrawHeader("LINE FOLLOWER", 1);

        {
            int kpI = (int)pid.Kp, kpF = (int)(fabsf(pid.Kp-(float)kpI)*10.0f);
            int kdI = (int)pid.Kd, kdF = (int)(fabsf(pid.Kd-(float)kdI)*10.0f);
            sprintf(buf, "Kp:%d.%d       Kd:%d.%d", kpI, kpF, kdI, kdF);
        }
        sh1106_SetCursor(0, 2); sh1106_WriteString(buf);

        {
            int kiI = (int)pid.Ki, kiF = (int)(fabsf(pid.Ki-(float)kiI)*100.0f);
            sprintf(buf, "Ki:%d.%02d      Spd:%d%%", kiI, kiF, pid.baseSpeed);
        }
        sh1106_SetCursor(0, 3); sh1106_WriteString(buf);

        sh1106_SetCursor(0, 5);
        sh1106_WriteString(calibrated ? "Cal: DONE" : "Cal: NOT DONE");

        sh1106_SetCursor(0, 6);
        sh1106_WriteString(">> PRESS E TO RUN <<");

        sh1106_SetCursor(0, 7); sh1106_WriteString("< >              E:go");

    } else if (mainCursor == 1) {
        DrawHeader("PID SETTINGS", 2);

        {int i=(int)pid.Kp, f=(int)(fabsf(pid.Kp-(float)i)*100); sprintf(buf,"  Kp  %d.%02d",i,f);}
        sh1106_SetCursor(0, 2); sh1106_WriteString(buf);
        {int i=(int)pid.Ki, f=(int)(fabsf(pid.Ki-(float)i)*100); sprintf(buf,"  Ki  %d.%02d",i,f);}
        sh1106_SetCursor(0, 3); sh1106_WriteString(buf);
        {int i=(int)pid.Kd, f=(int)(fabsf(pid.Kd-(float)i)*100); sprintf(buf,"  Kd  %d.%02d",i,f);}
        sh1106_SetCursor(0, 4); sh1106_WriteString(buf);
        sprintf(buf, "  Spd %d%%", pid.baseSpeed);
        sh1106_SetCursor(0, 5); sh1106_WriteString(buf);

        sh1106_SetCursor(0, 7); sh1106_WriteString("< >           E:edit");

    } else {
        DrawHeader("SENSOR DEBUG", 3);

        sh1106_SetCursor(0, 2); sh1106_WriteString("Raw 12-bit ADC values");
        sh1106_SetCursor(0, 3); sh1106_WriteString("* = on line");
        sh1106_SetCursor(0, 4); sh1106_WriteString(". = off line");

        sh1106_SetCursor(0, 7); sh1106_WriteString("< >              E:go");
    }
}

static void UI_DrawSensorDebug(void)
{
    sh1106_Clear();
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t chL = i;
        uint8_t chR = i + 8;
        char line[22];
        sprintf(line, "%2u:%04u%c  %2u:%04u%c",
                chL, sensorRaw[chL], sensorVal[chL] ? '*' : '.',
                chR, sensorRaw[chR], sensorVal[chR] ? '*' : '.');
        sh1106_SetCursor(0, i);
        sh1106_WriteString(line);
    }
}

static void UI_DrawRunning(void)
{
    sh1106_Clear();
    DrawHeader("RUNNING", 0);
    sh1106_SetCursor(78, 0); sh1106_WriteString("E:STOP");

    /* Sensor bar — all 16 sensors (ch=15..0, displayed left→right) */
    sh1106_SetCursor(4, 1);
    sh1106_WriteString("L ");
    for (int8_t ch = 15; ch >= 0; ch--)
        UI_WriteChar(sensorVal[ch] ? '*' : '.');
    sh1106_WriteString(" R");

    /* Position indicator bar (8px tall, row 2) */
    sh1106_DrawRect(0, 16, 127, 23);
    {
        int32_t px = ((int32_t)linePosition + (int32_t)SENSOR_POS_MAX)
                     * 122l / (2l * (int32_t)SENSOR_POS_MAX) + 3;
        if (px < 4)   px = 4;
        if (px > 123) px = 123;
        sh1106_FillRect((uint8_t)(px - 3), 17, (uint8_t)(px + 3), 22);
    }

    /* Direction */
    sh1106_SetCursor(0, 3);
    if      (linePosition < -(SENSOR_POS_MAX / 3)) sh1106_WriteString("<< LEFT             ");
    else if (linePosition >  (SENSOR_POS_MAX / 3)) sh1106_WriteString("            RIGHT >>");
    else                                            sh1106_WriteString("     [ CENTRED ]    ");

    /* Error + speed */
    {
        int  pos = linePosition >= 0 ? linePosition : -linePosition;
        char sgn = linePosition >= 0 ? '+' : '-';
        sprintf(buf, "Err:%c%-4d    Spd:%d%%", sgn, pos, pid.baseSpeed);
    }
    sh1106_SetCursor(0, 4);
    sh1106_WriteString(buf);

    /* Run time + health */
    {
        uint32_t sec = runElapsedMs / 1000u;
        uint32_t ten = (runElapsedMs % 1000u) / 100u;
        sprintf(buf, "T:%lu.%lus  Act:%d", sec, ten, sensorActiveCount);
    }
    sh1106_SetCursor(0, 5); sh1106_WriteString(buf);

    if (pidOscillating) {
        sh1106_SetCursor(0, 6); sh1106_WriteString("!! OSCILLATING !!");
    }

    sh1106_SetCursor(0, 7); sh1106_WriteString("Hold E to stop");
}

static void UI_DrawCalibrate(void)
{
    sh1106_Clear();

    if (calState == CAL_IDLE) {
        DrawHeader("CALIBRATE", 0);

        sh1106_SetCursor(0, 2); sh1106_WriteString("Place on track over");
        sh1106_SetCursor(0, 3); sh1106_WriteString("black and white.");
        sh1106_SetCursor(0, 5); sh1106_WriteString("Robot spins CW 5s.");

        sh1106_SetCursor(0, 7); sh1106_WriteString("E:start       L:back");

    } else {
        uint32_t ms  = Calibration_TimeRemaining();
        uint32_t sec = ms / 1000u;
        uint32_t ten = (ms % 1000u) / 100u;

        sh1106_SetCursor(0, 0); sh1106_WriteString("CALIBRATING...");
        sprintf(buf, "%lu.%lus", sec, ten);
        sh1106_SetCursor(93, 0); sh1106_WriteString(buf);
        sh1106_FillRect(0, 8, 127, 8);

        /* Sensor bar — all 16 sensors (ch=15..0, displayed left→right) */
        sh1106_SetCursor(4, 1);
        sh1106_WriteString("L ");
        for (int8_t ch = 15; ch >= 0; ch--)
            UI_WriteChar(sensorVal[ch] ? '*' : '.');
        sh1106_WriteString(" R");

        /* Confidence blocks — all 16 channels, 8 px each */
        for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
            uint8_t  ch    = 15u - i;
            uint16_t swing = (sensorCalMax[ch] > sensorCalMin[ch])
                             ? (sensorCalMax[ch] - sensorCalMin[ch]) : 0u;
            if (swing >= 600u)
                sh1106_FillRect((uint8_t)(i * 8), 16,
                                (uint8_t)(i * 8 + 6), 25);
        }

        uint8_t ready = Sensor_CalConfidence();
        sprintf(buf, "Ready: %d / 16", ready);
        sh1106_SetCursor(0, 4); sh1106_WriteString(buf);

        sh1106_SetCursor(0, 7); sh1106_WriteString("E / L = abort");
    }
}

static void UI_DrawPID(void)
{
    sh1106_Clear();
    DrawHeader("PID TUNING", 0);

    const char *labels[5] = {"Kp ", "Ki ", "Kd ", "Spd", "   "};
    uint8_t     rows[5]   = {2, 3, 4, 5, 6};

    for (uint8_t i = 0; i < 5; i++) {
        char cur = (i == pidCursor) ? ((pidEdit && i < 4) ? '*' : '>') : ' ';
        sh1106_SetCursor(0, rows[i]); UI_WriteChar(cur);
        sh1106_SetCursor(8, rows[i]);

        if (i < 3) {
            float *v[3] = {&pid.Kp, &pid.Ki, &pid.Kd};
            int ip = (int)*v[i], fp = (int)(fabsf(*v[i]-(float)ip)*100.0f);
            sprintf(buf, "%s %d.%02d", labels[i], ip, fp);
        } else if (i == 3) {
            sprintf(buf, "%s %d%%", labels[i], pid.baseSpeed);
        } else {
            sprintf(buf, "SAVE TO FLASH");
        }
        sh1106_WriteString(buf);
    }

    sh1106_SetCursor(0, 7);
    if      (pidCursor == 4) sh1106_WriteString("E = save to flash");
    else if (pidEdit)        sh1106_WriteString("L/R:change  E:done");
    else                     sh1106_WriteString("L/R:move    E:edit");
}

/* ================================================================
   PUBLIC API
   ================================================================ */

void UI_Init(void)
{
    sh1106_Init();
    sh1106_Clear();
    UI_Refresh();
}

void UI_Refresh(void)
{
    switch (currentScreen) {
        case SCR_MAIN:         UI_DrawMainMenu();     break;
        case SCR_RUNNING:      UI_DrawRunning();      break;
        case SCR_CALIBRATE:    UI_DrawCalibrate();    break;
        case SCR_PID:          UI_DrawPID();          break;
        case SCR_SENSOR_DEBUG: UI_DrawSensorDebug();  break;
    }
    sh1106_Display();
}
