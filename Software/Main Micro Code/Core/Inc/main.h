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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Test_Pin1_Pin GPIO_PIN_6
#define Test_Pin1_GPIO_Port GPIOA
#define Test_Pin2_Pin GPIO_PIN_7
#define Test_Pin2_GPIO_Port GPIOA
#define Test_Pin3_Pin GPIO_PIN_4
#define Test_Pin3_GPIO_Port GPIOC
#define Test_Pin4_Pin GPIO_PIN_5
#define Test_Pin4_GPIO_Port GPIOC
#define Test_Pin5_Pin GPIO_PIN_0
#define Test_Pin5_GPIO_Port GPIOB
#define Test_Pin6_Pin GPIO_PIN_1
#define Test_Pin6_GPIO_Port GPIOB
#define Test_Pin7_Pin GPIO_PIN_2
#define Test_Pin7_GPIO_Port GPIOB
#define Test_Pin8_Pin GPIO_PIN_10
#define Test_Pin8_GPIO_Port GPIOB
#define Test_Pin9_Pin GPIO_PIN_11
#define Test_Pin9_GPIO_Port GPIOB
#define IR_Sensor_IO1_Pin GPIO_PIN_6
#define IR_Sensor_IO1_GPIO_Port GPIOC
#define IR_Sensor_IO1_EXTI_IRQn EXTI4_15_IRQn
#define IR_Sensor_IO2_Pin GPIO_PIN_7
#define IR_Sensor_IO2_GPIO_Port GPIOC
#define IR_Sensor_IO2_EXTI_IRQn EXTI4_15_IRQn
#define Limit_SW1_Pin GPIO_PIN_9
#define Limit_SW1_GPIO_Port GPIOC
#define PUL_A_Pin GPIO_PIN_8
#define PUL_A_GPIO_Port GPIOA
#define DIR_A_Pin GPIO_PIN_9
#define DIR_A_GPIO_Port GPIOA
#define PUL_B_Pin GPIO_PIN_10
#define PUL_B_GPIO_Port GPIOA
#define DIR_B_Pin GPIO_PIN_11
#define DIR_B_GPIO_Port GPIOA
#define Limit_SW2_Pin GPIO_PIN_10
#define Limit_SW2_GPIO_Port GPIOC
#define Limit_SW3_Pin GPIO_PIN_11
#define Limit_SW3_GPIO_Port GPIOC
#define Limit_SW4_Pin GPIO_PIN_12
#define Limit_SW4_GPIO_Port GPIOC
#define DIR_IC_Pin GPIO_PIN_6
#define DIR_IC_GPIO_Port GPIOB
#define EN_A_Pin GPIO_PIN_7
#define EN_A_GPIO_Port GPIOB
#define EN_B_Pin GPIO_PIN_8
#define EN_B_GPIO_Port GPIOB
#define OE_Pin GPIO_PIN_9
#define OE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
