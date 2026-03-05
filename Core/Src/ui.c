/*
 * ui.c  —  UI/display rendering
 */

#include "ui.h"
#include "ssd1306.h"
#include "pid_controller.h"
#include "calibration.h"
#include "sensor.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ================================================================
   PRIVATE
   ================================================================ */

static char buf[32];

/* ================================================================
   HELPERS
   ================================================================ */

static void UI_WriteChar(char c)
{
    char tmp[2] = {c, '\0'};
    ssd1306_WriteString(tmp);
}

/* ================================================================
   SCREEN DRAWS
   ================================================================ */

static void UI_DrawMainMenu(void)
{
    ssd1306_Clear();

    if (mainCursor == 0) {
        /* ================================================
         * PAGE 1/4 — START RUN
         * ================================================ */
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("LINE FOLLOWER");
        ssd1306_SetCursor(90, 0);
        ssd1306_WriteString("[1/4]");
        ssd1306_DrawRect(0, 8, 127, 8);

        ssd1306_DrawRect(8, 18, 119, 31);
        ssd1306_SetCursor(14, 3);
        ssd1306_WriteString("  >>  START RUN  <<  ");

        sprintf(buf, "Spd:%d%%", pid.baseSpeed);
        ssd1306_SetCursor(0, 5);
        ssd1306_WriteString(buf);
        ssd1306_SetCursor(66, 5);
        ssd1306_WriteString(calibrated ? "Cal: DONE" : "Cal: NONE");

        {
            int kpI = (int)pid.Kp;
            int kpF = (int)(fabsf(pid.Kp - (float)kpI) * 10.0f);
            int kdI = (int)pid.Kd;
            int kdF = (int)(fabsf(pid.Kd - (float)kdI) * 10.0f);
            sprintf(buf, "Kp:%d.%d  Kd:%d.%d", kpI, kpF, kdI, kdF);
        }
        ssd1306_SetCursor(0, 6);
        ssd1306_WriteString(buf);

        ssd1306_SetCursor(0, 7);
        ssd1306_WriteString("< [1/4] >      E:GO");

    } else if (mainCursor == 1) {
        /* ================================================
         * PAGE 2/4 — CALIBRATE
         * ================================================ */
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("CALIBRATE");
        ssd1306_SetCursor(90, 0);
        ssd1306_WriteString("[2/4]");
        ssd1306_DrawRect(0, 8, 127, 8);

        ssd1306_SetCursor(0, 1);
        ssd1306_WriteString("1. Hover over WHITE");
        ssd1306_SetCursor(0, 2);
        ssd1306_WriteString("2. Press E to start");
        ssd1306_SetCursor(0, 3);
        ssd1306_WriteString("3. Sweep over BLACK");
        ssd1306_SetCursor(0, 4);
        ssd1306_WriteString("4. Press E when done");

        ssd1306_SetCursor(0, 6);
        ssd1306_WriteString(calibrated ? "Status: [CALIBRATED]" : "Status: [NOT DONE] ");

        ssd1306_SetCursor(0, 7);
        ssd1306_WriteString("< [2/4] >      E:GO");

    } else if (mainCursor == 2) {
        /* ================================================
         * PAGE 3/4 — PID PREVIEW
         * ================================================ */
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("PID SETTINGS");
        ssd1306_SetCursor(90, 0);
        ssd1306_WriteString("[3/4]");
        ssd1306_DrawRect(0, 8, 127, 8);

        {
            int i = (int)pid.Kp, f = (int)(fabsf(pid.Kp-(float)i)*100);
            sprintf(buf, " Kp : %d.%02d", i, f);
        }
        ssd1306_SetCursor(0, 2); ssd1306_WriteString(buf);

        {
            int i = (int)pid.Ki, f = (int)(fabsf(pid.Ki-(float)i)*100);
            sprintf(buf, " Ki : %d.%02d", i, f);
        }
        ssd1306_SetCursor(0, 3); ssd1306_WriteString(buf);

        {
            int i = (int)pid.Kd, f = (int)(fabsf(pid.Kd-(float)i)*100);
            sprintf(buf, " Kd : %d.%02d", i, f);
        }
        ssd1306_SetCursor(0, 4); ssd1306_WriteString(buf);

        sprintf(buf, " Speed : %d %%", pid.baseSpeed);
        ssd1306_SetCursor(0, 5); ssd1306_WriteString(buf);

        ssd1306_SetCursor(0, 7);
        ssd1306_WriteString("< [3/4] >    E:EDIT");

    } else {
        /* ================================================
         * PAGE 4/4 — SENSOR DEBUG
         * ================================================ */
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("SENSOR DEBUG");
        ssd1306_SetCursor(90, 0);
        ssd1306_WriteString("[4/4]");
        ssd1306_DrawRect(0, 8, 127, 8);

        ssd1306_SetCursor(0, 2);
        ssd1306_WriteString("View raw ADC values");
        ssd1306_SetCursor(0, 3);
        ssd1306_WriteString("for all 16 sensors.");
        ssd1306_SetCursor(0, 5);
        ssd1306_WriteString("*=on line  .=off");

        ssd1306_SetCursor(0, 7);
        ssd1306_WriteString("< [4/4] >      E:GO");
    }
}

static void UI_DrawSensorDebug(void)
{
    ssd1306_Clear();

    for (uint8_t i = 0; i < 8; i++) {
        uint8_t j = i + 8;
        char line[22];
        sprintf(line, "%2u:%04u%c %2u:%04u%c",
                i,  sensorRaw[i],  sensorVal[i]  ? '*' : '.',
                j,  sensorRaw[j],  sensorVal[j]  ? '*' : '.');
        ssd1306_SetCursor(0, (uint8_t)i);
        ssd1306_WriteString(line);
    }
}

static void UI_DrawRunning(void)
{
    ssd1306_Clear();

    ssd1306_SetCursor(4, 0);
    ssd1306_WriteString("RUNNING");
    ssd1306_SetCursor(78, 0);
    ssd1306_WriteString("E:STOP");
    ssd1306_DrawRect(0, 8, 127, 8);

    ssd1306_SetCursor(16, 1);
    for (uint8_t i = 0; i < 16; i++)
        UI_WriteChar(sensorVal[15 - i] ? '*' : '.');

    {
        ssd1306_DrawRect(0, 17, 127, 22);
        int32_t px = ((int32_t)linePosition + (int32_t)SENSOR_POS_MAX)
                     * 123l / (2l * (int32_t)SENSOR_POS_MAX) + 2;
        if (px < 2)   px = 2;
        if (px > 125) px = 125;
        ssd1306_FillRect((uint8_t)(px - 2), 18, (uint8_t)(px + 2), 21);
    }

    ssd1306_SetCursor(0, 3);
    if      (linePosition < -(SENSOR_POS_MAX / 3)) ssd1306_WriteString("<<  LEFT             ");
    else if (linePosition >  (SENSOR_POS_MAX / 3)) ssd1306_WriteString("            RIGHT  >>");
    else                                            ssd1306_WriteString("    [  CENTRED  ]    ");

    {
        int   pos = linePosition >= 0 ? linePosition : -linePosition;
        char  sgn = linePosition >= 0 ? '+' : '-';
        sprintf(buf, "Err:%c%-4d", sgn, pos);
    }
    ssd1306_SetCursor(0, 5);
    ssd1306_WriteString(buf);
    sprintf(buf, "Spd:%d%%", pid.baseSpeed);
    ssd1306_SetCursor(78, 5);
    ssd1306_WriteString(buf);

    ssd1306_SetCursor(0, 7);
    ssd1306_WriteString("Hold E = STOP");
}

static void UI_DrawCalibrate(void)
{
    ssd1306_Clear();

    if (calState == CAL_IDLE) {
        ssd1306_SetCursor(4, 0);
        ssd1306_WriteString("AUTO CALIBRATE");

        ssd1306_SetCursor(0, 2);
        ssd1306_WriteString("Place robot on track");
        ssd1306_SetCursor(0, 3);
        ssd1306_WriteString("then press E.");
        ssd1306_SetCursor(0, 5);
        ssd1306_WriteString("Robot will spin CW");
        ssd1306_SetCursor(0, 6);
        ssd1306_WriteString("for 5 s to calibrate.");

        ssd1306_SetCursor(0, 7);
        ssd1306_WriteString("E=start    L=back");
    } else {
        /* CAL_SPIN — show live sensor bar + countdown */
        ssd1306_SetCursor(16, 0);
        ssd1306_WriteString("CALIBRATING...");

        ssd1306_SetCursor(0, 1);
        ssd1306_WriteString("L I15         I0 R");

        ssd1306_SetCursor(16, 2);
        for (uint8_t i = 0; i < 8; i++)
            UI_WriteChar(sensorVal[15 - i] ? '*' : '.');
        ssd1306_SetCursor(16, 3);
        for (uint8_t i = 8; i < 16; i++)
            UI_WriteChar(sensorVal[15 - i] ? '*' : '.');

        ssd1306_DrawRect(0, 33, 127, 38);
        for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
            uint8_t ch = 15u - i;
            uint16_t swing = (sensorCalMax[ch] > sensorCalMin[ch])
                             ? (sensorCalMax[ch] - sensorCalMin[ch]) : 0u;
            if (swing >= 600u) {
                uint8_t x0 = (uint8_t)(i * 8);
                uint8_t x1 = (uint8_t)(x0 + 7);
                ssd1306_FillRect(x0, 34, x1, 37);
            }
        }

        uint8_t ready = Sensor_CalConfidence();
        sprintf(buf, "Ready:%2d/16", ready);
        ssd1306_SetCursor(0, 5);
        ssd1306_WriteString(buf);

        /* Countdown timer (tenths of a second) */
        uint32_t ms  = Calibration_TimeRemaining();
        uint32_t sec = ms / 1000u;
        uint32_t ten = (ms % 1000u) / 100u;
        sprintf(buf, "%lu.%lus left", sec, ten);
        ssd1306_SetCursor(66, 5);
        ssd1306_WriteString(buf);

        ssd1306_SetCursor(0, 7);
        ssd1306_WriteString("E/L = abort");
    }
}

static void UI_DrawPID(void)
{
    ssd1306_Clear();

    ssd1306_SetCursor(4, 0);
    ssd1306_WriteString("PID TUNING");
    ssd1306_DrawRect(0, 8, 127, 8);

    const char *labels[5] = {"Kp", "Ki", "Kd", "Spd", "SAV"};
    uint8_t     rows[5]   = {2, 3, 4, 5, 6};

    for (uint8_t i = 0; i < 5; i++) {
        char cur = ' ';
        if (i == pidCursor) cur = (pidEdit && i < 4) ? '*' : '>';
        ssd1306_SetCursor(0, rows[i]);
        UI_WriteChar(cur);
        ssd1306_SetCursor(6, rows[i]);
        if (i < 3) {
            float *v[3] = {&pid.Kp, &pid.Ki, &pid.Kd};
            float  val  = *v[i];
            int    ip   = (int)val;
            int    fp   = (int)(fabsf(val - (float)ip) * 100.0f);
            sprintf(buf, "%s : %d.%02d", labels[i], ip, fp);
        } else if (i == 3) {
            sprintf(buf, "%s: %d %%    ", labels[i], pid.baseSpeed);
        } else {
            sprintf(buf, "[ SAVE TO FLASH ]");
        }
        ssd1306_WriteString(buf);
    }

    ssd1306_SetCursor(0, 7);
    if (pidCursor == 4)
        ssd1306_WriteString("E = save to flash   ");
    else
        ssd1306_WriteString(pidEdit ? "L/R:change  E:done  " : "L/R:move  E:select  ");
}

/* ================================================================
   PUBLIC API
   ================================================================ */

void UI_Init(void)
{
    ssd1306_Init();
    ssd1306_Clear();
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
    ssd1306_Display();
}
