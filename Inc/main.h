/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define SENSOR_LS_Pin GPIO_PIN_0
#define SENSOR_LS_GPIO_Port GPIOA
#define SENSOR_L1_Pin GPIO_PIN_1
#define SENSOR_L1_GPIO_Port GPIOA
#define SENSOR_L2_Pin GPIO_PIN_4
#define SENSOR_L2_GPIO_Port GPIOA
#define SENSOR_R1_Pin GPIO_PIN_5
#define SENSOR_R1_GPIO_Port GPIOA
#define SENSOR_R2_Pin GPIO_PIN_6
#define SENSOR_R2_GPIO_Port GPIOA
#define SENSOR_RS_Pin GPIO_PIN_7
#define SENSOR_RS_GPIO_Port GPIOA
#define SENSOR_S1_Pin GPIO_PIN_0
#define SENSOR_S1_GPIO_Port GPIOB
#define SENSOR_S2_Pin GPIO_PIN_1
#define SENSOR_S2_GPIO_Port GPIOB
#define R_PWM_Pin GPIO_PIN_10
#define R_PWM_GPIO_Port GPIOB
#define RGB_Pin GPIO_PIN_12
#define RGB_GPIO_Port GPIOB
#define RGB_BLUE_Pin GPIO_PIN_13
#define RGB_BLUE_GPIO_Port GPIOB
#define R_SILNIK1_Pin GPIO_PIN_8
#define R_SILNIK1_GPIO_Port GPIOA
#define R_SILNIK2_Pin GPIO_PIN_9
#define R_SILNIK2_GPIO_Port GPIOA
#define L_SILNIK2_Pin GPIO_PIN_3
#define L_SILNIK2_GPIO_Port GPIOB
#define L_PWM_Pin GPIO_PIN_4
#define L_PWM_GPIO_Port GPIOB
#define L_SILNIK1_Pin GPIO_PIN_5
#define L_SILNIK1_GPIO_Port GPIOB
#define DIODA_IR_Pin GPIO_PIN_6
#define DIODA_IR_GPIO_Port GPIOB
#define DIODA_IR_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
