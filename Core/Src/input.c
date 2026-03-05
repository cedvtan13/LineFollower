/*
 * input.c  —  Button input handling and debouncing
 */

#include "input.h"
#include "main.h"
#include "stm32f4xx_hal.h"

/* ================================================================
   CONFIGURATION
   ================================================================ */

#define BTN_DEBOUNCE_MS  200u
#define BTN_LONG_MS     1000u

/* ================================================================
   PRIVATE STATE
   ================================================================ */

static uint32_t lastL      = 0;
static uint32_t lastR      = 0;
static uint32_t enterDown  = 0;
static uint8_t  enterHeld  = 0;
static ButtonEvent lastEvent = BTN_NONE;

/* ================================================================
   IMPLEMENTATION
   ================================================================ */

void Input_Init(void)
{
    lastL      = 0;
    lastR      = 0;
    enterDown  = 0;
    enterHeld  = 0;
    lastEvent  = BTN_NONE;
}

void Input_Update(void)
{
    lastEvent = BTN_NONE;  /* clear previous event */

    uint32_t now = HAL_GetTick();
    uint8_t  L   = !HAL_GPIO_ReadPin(L_But_GPIO_Port, L_But_Pin);
    uint8_t  E   = !HAL_GPIO_ReadPin(E_But_GPIO_Port, E_But_Pin);
    uint8_t  R   = !HAL_GPIO_ReadPin(R_But_GPIO_Port, R_But_Pin);

    /* ---- Enter button (short / long press) ---- */
    if (E) {
        if (enterDown == 0) {
            /* Use (now | 1) so tick=0 never masquerades as "not pressed" */
            enterDown = now ? now : 1u;
            enterHeld = 0;
        } else if (!enterHeld && (now - enterDown) >= BTN_LONG_MS) {
            enterHeld = 1;
            lastEvent = BTN_ENTER_LONG;
        }
    } else {
        if (enterDown != 0 && !enterHeld) {
            lastEvent = BTN_ENTER_SHORT;
        }
        enterDown = 0;
        enterHeld = 0;
    }

    /* ---- Left / Right buttons ----
     * Only update lastEvent when no higher-priority E event was already
     * set this iteration, so a long-press cannot be silently overwritten
     * if L or R happens to be physically held at the same time.
     */
    if (lastEvent == BTN_NONE) {
        if (L && (now - lastL) >= BTN_DEBOUNCE_MS) {
            lastL     = now;
            lastEvent = BTN_LEFT;
        } else if (R && (now - lastR) >= BTN_DEBOUNCE_MS) {
            lastR     = now;
            lastEvent = BTN_RIGHT;
        }
    }
}

ButtonEvent Input_GetEvent(void)
{
    ButtonEvent ev = lastEvent;
    lastEvent = BTN_NONE;
    return ev;
}
