/*
 * sh1106.h  
 * Driver for SH1106 1.3" 128x64 OLED over I2C.
 *
 *  Created on: Feb 27, 2026
 *      Author: cvt
 */

#ifndef INC_SH1106_H_
#define INC_SH1106_H_

#include "main.h"
/* SH1106_ADDR defined in sh1106.c — 0x78 (0x3C<<1) */

void sh1106_Init(void);
void sh1106_SendCommand(uint8_t cmd);
void sh1106_SendData(uint8_t data);
void sh1106_SetCursor(uint8_t x, uint8_t y);
void sh1106_WriteString(const char *str);
void sh1106_Display(void);
void sh1106_Clear(void);

/* Pixel / shape drawing — use with sh1106_Display() to flush */
void sh1106_DrawPixel(uint8_t x, uint8_t y, uint8_t on);
void sh1106_DrawRect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void sh1106_FillRect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);



#endif /* INC_SH1106_H_ */
