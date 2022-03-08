/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include "string.h"
#include "Timer_Delay.h"
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
#define PWM_SERWO_Pin GPIO_PIN_0
#define PWM_SERWO_GPIO_Port GPIOA
#define LIMIT_J2_Pin GPIO_PIN_5
#define LIMIT_J2_GPIO_Port GPIOA
#define LIMIT_Z_Pin GPIO_PIN_6
#define LIMIT_Z_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_7
#define ENABLE_GPIO_Port GPIOA
#define STEP2_Pin GPIO_PIN_10
#define STEP2_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_7
#define RESET_GPIO_Port GPIOC
#define STEP1_Pin GPIO_PIN_8
#define STEP1_GPIO_Port GPIOA
#define SLEEP_Pin GPIO_PIN_9
#define SLEEP_GPIO_Port GPIOA
#define DIR3_Pin GPIO_PIN_10
#define DIR3_GPIO_Port GPIOA
#define DIR2_Pin GPIO_PIN_3
#define DIR2_GPIO_Port GPIOB
#define STEP3_Pin GPIO_PIN_4
#define STEP3_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_5
#define DIR1_GPIO_Port GPIOB
#define M_1_2_3_Pin GPIO_PIN_6
#define M_1_2_3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
