/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BIN1_Pin GPIO_PIN_14
#define BIN1_GPIO_Port GPIOC
#define BIN2_Pin GPIO_PIN_15
#define BIN2_GPIO_Port GPIOC
#define PWMA_Pin GPIO_PIN_0
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_1
#define PWMB_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_2
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_3
#define AIN2_GPIO_Port GPIOA
#define STNBY_Pin GPIO_PIN_4
#define STNBY_GPIO_Port GPIOA
/* CD74HC4067SM 16-channel MUX channel-select outputs:
 *   S0 = PB0   S1 = PA7   S2 = PA6   S3 = PA5
 * SIG = PB1  (analog input, ADC1_CH9 — no GPIO define needed)
 * I0 = rightmost sensor, I15 = leftmost sensor */
#define MUX_S0_Pin GPIO_PIN_0
#define MUX_S0_GPIO_Port GPIOB
#define MUX_S1_Pin GPIO_PIN_7
#define MUX_S1_GPIO_Port GPIOA
#define MUX_S2_Pin GPIO_PIN_6
#define MUX_S2_GPIO_Port GPIOA
#define MUX_S3_Pin GPIO_PIN_5
#define MUX_S3_GPIO_Port GPIOA
#define LED_Indi_Pin GPIO_PIN_10
#define LED_Indi_GPIO_Port GPIOB
#define L_But_Pin GPIO_PIN_12
#define L_But_GPIO_Port GPIOB
#define E_But_Pin GPIO_PIN_13
#define E_But_GPIO_Port GPIOB
#define R_But_Pin GPIO_PIN_14
#define R_But_GPIO_Port GPIOB
#define ACCEL_SCL_Pin GPIO_PIN_8
#define ACCEL_SCL_GPIO_Port GPIOA
#define ACCEL_SDA_Pin GPIO_PIN_4
#define ACCEL_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
