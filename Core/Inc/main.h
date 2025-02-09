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
#include "stm32g4xx_hal.h"

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
void StartDefaultTask(void *argument);

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//void softwareReset();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define V_CHASSIS_Pin GPIO_PIN_1
#define V_CHASSIS_GPIO_Port GPIOC
#define I_SOURCE_Pin GPIO_PIN_2
#define I_SOURCE_GPIO_Port GPIOC
#define I_CHASSIS_Pin GPIO_PIN_3
#define I_CHASSIS_GPIO_Port GPIOC
#define I_CAP_Pin GPIO_PIN_0
#define I_CAP_GPIO_Port GPIOA
#define V_CAP_Pin GPIO_PIN_1
#define V_CAP_GPIO_Port GPIOA
#define V_TEMP_Pin GPIO_PIN_1
#define V_TEMP_GPIO_Port GPIOB
#define GPIO_LED_Pin GPIO_PIN_6
#define GPIO_LED_GPIO_Port GPIOC
#define BAT_MOS_LOW_Pin GPIO_PIN_8
#define BAT_MOS_LOW_GPIO_Port GPIOC
#define BAT_MOS_HIGH_Pin GPIO_PIN_9
#define BAT_MOS_HIGH_GPIO_Port GPIOC
#define CAP_MOS_LOW_Pin GPIO_PIN_8
#define CAP_MOS_LOW_GPIO_Port GPIOA
#define CAP_MOS_HIGH_Pin GPIO_PIN_9
#define CAP_MOS_HIGH_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
//#define CCMRAM __attribute__((section (".ccmram")))
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
