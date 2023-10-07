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
#include "stm32l4xx_hal.h"

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
#define LCD_BL_Pin GPIO_PIN_1
#define LCD_BL_GPIO_Port GPIOA
#define LCD_BL1_Pin GPIO_PIN_2
#define LCD_BL1_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_0
#define LED_RED_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_1
#define LED_BLUE_GPIO_Port GPIOB
#define PHOTO_Pin GPIO_PIN_8
#define PHOTO_GPIO_Port GPIOA
#define PHOTO_EXTI_IRQn EXTI9_5_IRQn
#define LED_ORANGE_Pin GPIO_PIN_3
#define LED_ORANGE_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_4
#define LCD_DC_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_5
#define LCD_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
