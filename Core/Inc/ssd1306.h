/*
 * sh1106.h  (kept as ssd1306.h for compatibility)
 * Driver for SH1106 1.3" 128x64 OLED over I2C.
 *
 *  Created on: Feb 27, 2026
 *      Author: cvt
 */

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_

#include "main.h"
/* SSD1306_ADDR is defined in ssd1306.c — 0x78 (0x3C<<1) */

void ssd1306_Init(void);
void ssd1306_SendCommand(uint8_t cmd);
void ssd1306_SendData(uint8_t data);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
void ssd1306_WriteString(char *str);
void ssd1306_Display(void);
void ssd1306_Clear(void);

/* Pixel / shape drawing — use with ssd1306_Display() to flush */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, uint8_t on);
void ssd1306_DrawRect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ssd1306_FillRect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);



#endif /* INC_SSD1306_H_ */
