/*
 * ui.h  —  UI/display rendering
 */

#ifndef UI_H
#define UI_H

#include <stdint.h>
#include "menu.h"  /* AppScreen, currentScreen, etc. */

extern uint8_t   mainCursor;
extern uint8_t   pidCursor;
extern uint8_t   pidEdit;

/* Initialize display */
void UI_Init(void);

/* Refresh the current screen (redraw to framebuffer) */
void UI_Refresh(void);

#endif
