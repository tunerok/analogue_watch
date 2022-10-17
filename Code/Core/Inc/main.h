/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DS1302.h"
#include "MCP4725.h"
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
#define CLOCK_CLK_Pin GPIO_PIN_5
#define CLOCK_CLK_GPIO_Port GPIOA
#define CLOCK_nRST_Pin GPIO_PIN_6
#define CLOCK_nRST_GPIO_Port GPIOA
#define CLOCK_DATA_Pin GPIO_PIN_7
#define CLOCK_DATA_GPIO_Port GPIOA
#define LED_MIN_Pin GPIO_PIN_10
#define LED_MIN_GPIO_Port GPIOB
#define LED_HOUR_Pin GPIO_PIN_11
#define LED_HOUR_GPIO_Port GPIOB
#define LED_SEC_Pin GPIO_PIN_12
#define LED_SEC_GPIO_Port GPIOB
#define BTN_SET_LED_INP_Pin GPIO_PIN_14
#define BTN_SET_LED_INP_GPIO_Port GPIOB
#define BTN_MENU_INP_Pin GPIO_PIN_15
#define BTN_MENU_INP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define BTN_HOLD_DELAY				1000
#define BTN_DEBOUNCER_MS			50
#define LED_BLINK_TICK_MS_TIMES_10 	25
#define MAX_VOLTAGE_TO_SHOW			3
#define VOLTAGE_CORR_COEFF			1.03
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
