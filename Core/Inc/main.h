/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define SOUNDER_SEL1_Pin GPIO_PIN_13
#define SOUNDER_SEL1_GPIO_Port GPIOC
#define EN_3V3_SW_Pin GPIO_PIN_15
#define EN_3V3_SW_GPIO_Port GPIOC
#define WDT_EN_Pin GPIO_PIN_14
#define WDT_EN_GPIO_Port GPIOF
#define WDT_IP_Pin GPIO_PIN_15
#define WDT_IP_GPIO_Port GPIOF
#define LED_POWER_RED_Pin GPIO_PIN_0
#define LED_POWER_RED_GPIO_Port GPIOG
#define LED_POWER_GRN_Pin GPIO_PIN_1
#define LED_POWER_GRN_GPIO_Port GPIOG
#define LED_GAS_RED_Pin GPIO_PIN_7
#define LED_GAS_RED_GPIO_Port GPIOE
#define LED_GAS_GRN_Pin GPIO_PIN_8
#define LED_GAS_GRN_GPIO_Port GPIOE
#define LED_FAULT_RED_Pin GPIO_PIN_9
#define LED_FAULT_RED_GPIO_Port GPIOE
#define LED_FAULT_GRN_Pin GPIO_PIN_10
#define LED_FAULT_GRN_GPIO_Port GPIOE
#define LED_COMMS_BLUE_Pin GPIO_PIN_11
#define LED_COMMS_BLUE_GPIO_Port GPIOE
#define POWER_BTN_Pin GPIO_PIN_12
#define POWER_BTN_GPIO_Port GPIOE
#define LED_RED_POD4_Pin GPIO_PIN_10
#define LED_RED_POD4_GPIO_Port GPIOD
#define LED_GRN_POD4_Pin GPIO_PIN_11
#define LED_GRN_POD4_GPIO_Port GPIOD
#define LED_GRN_POD3_Pin GPIO_PIN_4
#define LED_GRN_POD3_GPIO_Port GPIOG
#define LED_RED_POD3_Pin GPIO_PIN_5
#define LED_RED_POD3_GPIO_Port GPIOG
#define LED_GRN_POD2_Pin GPIO_PIN_9
#define LED_GRN_POD2_GPIO_Port GPIOC
#define LED_RED_POD2_Pin GPIO_PIN_8
#define LED_RED_POD2_GPIO_Port GPIOA
#define LED_GRN_POD1_Pin GPIO_PIN_10
#define LED_GRN_POD1_GPIO_Port GPIOG
#define LED_RED_POD1_Pin GPIO_PIN_11
#define LED_RED_POD1_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
