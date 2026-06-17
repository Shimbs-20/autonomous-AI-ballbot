/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Black_Pill_LED_Pin GPIO_PIN_13
#define Black_Pill_LED_GPIO_Port GPIOC
#define ENAPLE_M3_Pin GPIO_PIN_14
#define ENAPLE_M3_GPIO_Port GPIOC
#define ENAPLE_M2_Pin GPIO_PIN_1
#define ENAPLE_M2_GPIO_Port GPIOB
#define M1_STEP_Pin GPIO_PIN_12
#define M1_STEP_GPIO_Port GPIOB
#define M1_DIR_Pin GPIO_PIN_13
#define M1_DIR_GPIO_Port GPIOB
#define M2_STEP_Pin GPIO_PIN_14
#define M2_STEP_GPIO_Port GPIOB
#define M2_DIR_Pin GPIO_PIN_15
#define M2_DIR_GPIO_Port GPIOB
#define M3_STEP_Pin GPIO_PIN_8
#define M3_STEP_GPIO_Port GPIOA
#define M3_DIR_Pin GPIO_PIN_9
#define M3_DIR_GPIO_Port GPIOA
#define ENABLE_PIN_Pin GPIO_PIN_10
#define ENABLE_PIN_GPIO_Port GPIOA
#define M3_DIR1_Pin GPIO_PIN_15
#define M3_DIR1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
