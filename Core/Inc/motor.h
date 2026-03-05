/*
 * motor.h  —  TB6612FNG dual motor driver
 *
 * Motor A (LEFT) : AIN1=PA2  AIN2=PA3  PWMA=TIM2_CH1 (PA0)
 * Motor B (RIGHT): BIN1=PC14 BIN2=PC15 PWMB=TIM2_CH2 (PA1)
 * STBY           : PA4  (pull LOW to coast, pull HIGH to enable)
 *
 * PWM: 100 MHz / (4+1) / (999+1) = 20 kHz, 1000 steps (0–1000)
 *
 * N20 1000 RPM 12 V deadband:
 *   Below ~15% duty the gearbox won't spin.
 *   Motor_SetSpeeds() accepts 0–100% and maps through a deadband
 *   so 1% yields the minimum tick that overcomes stiction.
 *   Adjust MOTOR_DEADBAND in motor.c if your motor behaves differently.
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include <stdint.h>

/*
 * Speed in -100..+100 %.
 * Positive  → motor spins forward.
 * Negative  → motor spins in reverse (pivot / tank turn).
 * Zero      → coast.
 */
void Motor_SetSpeeds(int16_t leftPct, int16_t rightPct);

/* Start PWM timers and assert STBY (call before Motor_SetSpeeds) */
void Motor_Enable(void);

/* Zero PWM and de-assert STBY (coast) */
void Motor_Stop(void);

/*
 * Motor_HardBrake: TB6612FNG short-circuit brake.
 * AIN1=AIN2=HIGH, BIN1=BIN2=HIGH  → both motor outputs shorted to GND.
 * Much sharper deceleration than coast.  Use for pivot recovery.
 */
void Motor_HardBrake(void);

#endif /* INC_MOTOR_H_ */
