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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOC
#define AIN2_Pin GPIO_PIN_1
#define AIN2_GPIO_Port GPIOC
#define BIN1_Pin GPIO_PIN_2
#define BIN1_GPIO_Port GPIOC
#define BIN2_Pin GPIO_PIN_3
#define BIN2_GPIO_Port GPIOC
#define ED1_Pin GPIO_PIN_0
#define ED1_GPIO_Port GPIOA
#define ED2_Pin GPIO_PIN_1
#define ED2_GPIO_Port GPIOA
#define OLED_SDA_Pin GPIO_PIN_2
#define OLED_SDA_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_3
#define OLED_SCL_GPIO_Port GPIOA
#define EB2_Pin GPIO_PIN_7
#define EB2_GPIO_Port GPIOA
#define CIN1_Pin GPIO_PIN_4
#define CIN1_GPIO_Port GPIOC
#define CIN2_Pin GPIO_PIN_5
#define CIN2_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_9
#define PWMA_GPIO_Port GPIOE
#define PWMB_Pin GPIO_PIN_11
#define PWMB_GPIO_Port GPIOE
#define PWMC_Pin GPIO_PIN_13
#define PWMC_GPIO_Port GPIOE
#define PWMD_Pin GPIO_PIN_14
#define PWMD_GPIO_Port GPIOE
#define EC1_Pin GPIO_PIN_12
#define EC1_GPIO_Port GPIOD
#define EC2_Pin GPIO_PIN_13
#define EC2_GPIO_Port GPIOD
#define EA1_Pin GPIO_PIN_6
#define EA1_GPIO_Port GPIOC
#define EA2_Pin GPIO_PIN_7
#define EA2_GPIO_Port GPIOC
#define DIN1_Pin GPIO_PIN_3
#define DIN1_GPIO_Port GPIOD
#define DIN2_Pin GPIO_PIN_5
#define DIN2_GPIO_Port GPIOD
#define EB1_Pin GPIO_PIN_4
#define EB1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
