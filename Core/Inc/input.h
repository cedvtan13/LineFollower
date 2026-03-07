/*
 * input.h  —  button input handling
 */

#ifndef INPUT_H
#define INPUT_H

typedef enum {
    BTN_NONE = 0,
    BTN_LEFT,
    BTN_RIGHT,
    BTN_ENTER_SHORT,
    BTN_ENTER_LONG,
} ButtonEvent;

void Input_Init(void);
void Input_Update(void);
ButtonEvent Input_GetEvent(void);

#endif