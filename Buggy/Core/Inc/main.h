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
// States
#define straight 			1
#define maneuver			2
#define wait_for_echo_low	3
#define wait_for_echo_high	4
#define measurement_done	5




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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Dir_B_Pin GPIO_PIN_5
#define Dir_B_GPIO_Port GPIOA
#define Dir_A_Pin GPIO_PIN_6
#define Dir_A_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_7
#define PWM_B_GPIO_Port GPIOA
#define Trigger_Pin GPIO_PIN_14
#define Trigger_GPIO_Port GPIOB
#define Echo_Pin GPIO_PIN_15
#define Echo_GPIO_Port GPIOB
#define Echo_EXTI_IRQn EXTI15_10_IRQn
#define Brake_A_Pin GPIO_PIN_7
#define Brake_A_GPIO_Port GPIOC
#define BNO055_Activation_Pin GPIO_PIN_8
#define BNO055_Activation_GPIO_Port GPIOA
#define Brake_B_Pin GPIO_PIN_9
#define Brake_B_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define PWM_A_Pin GPIO_PIN_3
#define PWM_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
