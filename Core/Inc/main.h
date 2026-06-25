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
#include "stm32f1xx_hal.h"

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
#define LED_Board_Pin GPIO_PIN_13
#define LED_Board_GPIO_Port GPIOC
#define Servo_Pin GPIO_PIN_0
#define Servo_GPIO_Port GPIOA
#define Pump_Pin GPIO_PIN_6
#define Pump_GPIO_Port GPIOA
#define Sound_8_Pin GPIO_PIN_15
#define Sound_8_GPIO_Port GPIOA
#define Sound_7_Pin GPIO_PIN_3
#define Sound_7_GPIO_Port GPIOB
#define Sound_6_Pin GPIO_PIN_4
#define Sound_6_GPIO_Port GPIOB
#define Sound_5_Pin GPIO_PIN_5
#define Sound_5_GPIO_Port GPIOB
#define Sound_4_Pin GPIO_PIN_6
#define Sound_4_GPIO_Port GPIOB
#define Sound_3_Pin GPIO_PIN_7
#define Sound_3_GPIO_Port GPIOB
#define Sound_2_Pin GPIO_PIN_8
#define Sound_2_GPIO_Port GPIOB
#define Sound_1_Pin GPIO_PIN_9
#define Sound_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
