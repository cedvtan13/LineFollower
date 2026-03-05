/*
 * input.h  —  Button input handling and debouncing
 */

#ifndef INPUT_H
#define INPUT_H

#include <stdint.h>

/* Button state and callbacks */
typedef enum {
    BTN_NONE,
    BTN_LEFT,
    BTN_RIGHT,
    BTN_ENTER_SHORT,
    BTN_ENTER_LONG,
} ButtonEvent;

/* Initialize input state */
void Input_Init(void);

/* Update button state (call every loop) */
void Input_Update(void);

/* Get the latest button event (returns BTN_NONE if no new event) */
ButtonEvent Input_GetEvent(void);

#endif
