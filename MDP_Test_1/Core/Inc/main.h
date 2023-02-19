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
#define OLED_SCL_Pin GPIO_PIN_5
#define OLED_SCL_GPIO_Port GPIOE
#define OLED_SDA_Pin GPIO_PIN_6
#define OLED_SDA_GPIO_Port GPIOE
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define MOTORA_IN2_Pin GPIO_PIN_2
#define MOTORA_IN2_GPIO_Port GPIOA
#define MOTORA_IN1_Pin GPIO_PIN_3
#define MOTORA_IN1_GPIO_Port GPIOA
#define MOTORB_IN1_Pin GPIO_PIN_4
#define MOTORB_IN1_GPIO_Port GPIOA
#define MOTORB_IN2_Pin GPIO_PIN_5
#define MOTORB_IN2_GPIO_Port GPIOA
#define MOTORB_ENCA_Pin GPIO_PIN_6
#define MOTORB_ENCA_GPIO_Port GPIOA
#define MOTORB_ENCB_Pin GPIO_PIN_7
#define MOTORB_ENCB_GPIO_Port GPIOA
#define OLED_RES_Pin GPIO_PIN_7
#define OLED_RES_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_8
#define OLED_DC_GPIO_Port GPIOE
#define LED_OUTn_Pin GPIO_PIN_10
#define LED_OUTn_GPIO_Port GPIOE
#define SERVO_PWM_Pin GPIO_PIN_14
#define SERVO_PWM_GPIO_Port GPIOE
#define MOTORA_PWM_Pin GPIO_PIN_6
#define MOTORA_PWM_GPIO_Port GPIOC
#define MOTORB_PWM_Pin GPIO_PIN_7
#define MOTORB_PWM_GPIO_Port GPIOC
#define MOTORA_ENCA_Pin GPIO_PIN_15
#define MOTORA_ENCA_GPIO_Port GPIOA
#define MOTORA_ENCB_Pin GPIO_PIN_3
#define MOTORA_ENCB_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_8
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_9
#define IMU_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
