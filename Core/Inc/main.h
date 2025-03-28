/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define LED_0_Pin GPIO_PIN_13
#define LED_0_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_14
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_15
#define LED_2_GPIO_Port GPIOC
#define EX_GPIO_4_Pin GPIO_PIN_0
#define EX_GPIO_4_GPIO_Port GPIOA
#define SW_3_Pin GPIO_PIN_3
#define SW_3_GPIO_Port GPIOA
#define LED_3_Pin GPIO_PIN_6
#define LED_3_GPIO_Port GPIOA
#define LED_4_Pin GPIO_PIN_7
#define LED_4_GPIO_Port GPIOA
#define SW_2_Pin GPIO_PIN_12
#define SW_2_GPIO_Port GPIOB
#define PHOTO_0_Pin GPIO_PIN_13
#define PHOTO_0_GPIO_Port GPIOB
#define PHOTO_1_Pin GPIO_PIN_14
#define PHOTO_1_GPIO_Port GPIOB
#define SW_0_Pin GPIO_PIN_15
#define SW_0_GPIO_Port GPIOB
#define SW_1_Pin GPIO_PIN_8
#define SW_1_GPIO_Port GPIOA
#define EX_GPIO_3_Pin GPIO_PIN_6
#define EX_GPIO_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
