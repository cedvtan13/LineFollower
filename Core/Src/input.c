/*
 * input.c  —  simple button handling
 */

#include "input.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#define BTN_DEBOUNCE_MS 180u
#define BTN_LONG_MS    900u

static uint32_t lastL = 0;
static uint32_t lastR = 0;
static uint32_t enterDown = 0;
static uint8_t enterHeld = 0;
static ButtonEvent lastEvent = BTN_NONE;

void Input_Init(void)
{
    lastL = 0;
    lastR = 0;
    enterDown = 0;
    enterHeld = 0;
    lastEvent = BTN_NONE;
}

void Input_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t L = (uint8_t)!HAL_GPIO_ReadPin(L_But_GPIO_Port, L_But_Pin);
    uint8_t E = (uint8_t)!HAL_GPIO_ReadPin(E_But_GPIO_Port, E_But_Pin);
    uint8_t R = (uint8_t)!HAL_GPIO_ReadPin(R_But_GPIO_Port, R_But_Pin);

    lastEvent = BTN_NONE;

    if (E) {
        if (enterDown == 0u) {
            enterDown = (now == 0u) ? 1u : now;
            enterHeld = 0u;
        } else if (!enterHeld && (now - enterDown) >= BTN_LONG_MS) {
            enterHeld = 1u;
            lastEvent = BTN_ENTER_LONG;
        }
    } else {
        if (enterDown != 0u && !enterHeld) lastEvent = BTN_ENTER_SHORT;
        enterDown = 0u;
        enterHeld = 0u;
    }

    if (lastEvent == BTN_NONE) {
        if (L && (now - lastL) >= BTN_DEBOUNCE_MS) {
            lastL = now;
            lastEvent = BTN_LEFT;
        } else if (R && (now - lastR) >= BTN_DEBOUNCE_MS) {
            lastR = now;
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