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
#include "stm32f0xx_hal.h"

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
#define LED_UART_INDICATOR_Pin GPIO_PIN_13
#define LED_UART_INDICATOR_GPIO_Port GPIOC
#define USART2_TX_ESP_Pin GPIO_PIN_2
#define USART2_TX_ESP_GPIO_Port GPIOA
#define USART2_RX_ESP_Pin GPIO_PIN_3
#define USART2_RX_ESP_GPIO_Port GPIOA
#define LED_TX_Pin GPIO_PIN_4
#define LED_TX_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_0
#define SD_CS_GPIO_Port GPIOB
#define RS485_DE_RE_Pin GPIO_PIN_8
#define RS485_DE_RE_GPIO_Port GPIOA
#define USART1_TX_RS485_Pin GPIO_PIN_9
#define USART1_TX_RS485_GPIO_Port GPIOA
#define USART1_RX_RS485_Pin GPIO_PIN_10
#define USART1_RX_RS485_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
