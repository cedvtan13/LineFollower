/*
 * motor.c  —  TB6612FNG dual motor driver
 */

#include "motor.h"

extern TIM_HandleTypeDef htim2;

/* PWM resolution matches TIM2 ARR = 999 → 1000 steps (0–1000) */
#define PWM_MAX      1000
/* Minimum duty tick to overcome N20 1000 RPM 12 V gearbox stiction (~15%) */
#define MOTOR_DEADBAND 150

/*
 * SpeedToPWM: map user percentage (0–100) to TIM compare value.
 *   0%   → 0           (full coast)
 *   1%   → DEADBAND    (just enough to spin)
 *   100% → PWM_MAX
 */
static uint16_t SpeedToPWM(uint16_t pct)
{
    if (pct == 0)    return 0;
    if (pct > 100u)  pct = 100u;
    /* Explicit 32-bit intermediate prevents any risk of 16-bit overflow */
    return (uint16_t)(MOTOR_DEADBAND + ((uint32_t)pct * (PWM_MAX - MOTOR_DEADBAND)) / 100u);
}

void Motor_SetSpeeds(int16_t leftPct, int16_t rightPct)
{
    /* --- Left motor (Motor A) direction --- */
    if (leftPct > 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else if (leftPct < 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);  /* coast */
    }

    /* --- Right motor (Motor B) direction --- */
    if (rightPct > 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    } else if (rightPct < 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);  /* coast */
    }

    /* Compute absolute values once — avoids repeating the ternary at the call site */
    uint16_t lAbs = (uint16_t)(leftPct  < 0 ? -leftPct  : leftPct);
    uint16_t rAbs = (uint16_t)(rightPct < 0 ? -rightPct : rightPct);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SpeedToPWM(lAbs));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SpeedToPWM(rAbs));
}

void Motor_Enable(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(STNBY_GPIO_Port, STNBY_Pin, GPIO_PIN_SET);
}

void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(STNBY_GPIO_Port, STNBY_Pin, GPIO_PIN_RESET);
}

void Motor_HardBrake(void)
{
    /* TB6612FNG: AIN1=AIN2=1, BIN1=BIN2=1 → short-circuit brake both motors */
    /* PWM to 0 first so the bridge isn't fighting itself */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(STNBY_GPIO_Port, STNBY_Pin, GPIO_PIN_SET);
}
