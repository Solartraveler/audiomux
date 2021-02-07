/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

extern ADC_HandleTypeDef hadc;

extern IWDG_HandleTypeDef hiwdg;

extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Crystal0_Pin GPIO_PIN_0
#define Crystal0_GPIO_Port GPIOF
#define Crystal1_Pin GPIO_PIN_1
#define Crystal1_GPIO_Port GPIOF
#define SoundA1_Pin GPIO_PIN_0
#define SoundA1_GPIO_Port GPIOA
#define SoundA2_Pin GPIO_PIN_1
#define SoundA2_GPIO_Port GPIOA
#define SoundA3_Pin GPIO_PIN_4
#define SoundA3_GPIO_Port GPIOA
#define SoundA4_Pin GPIO_PIN_5
#define SoundA4_GPIO_Port GPIOA
#define SoundAMute_Pin GPIO_PIN_6
#define SoundAMute_GPIO_Port GPIOA
#define SoundB1_Pin GPIO_PIN_7
#define SoundB1_GPIO_Port GPIOA
#define JackVcc_Pin GPIO_PIN_0
#define JackVcc_GPIO_Port GPIOB
#define UsbVcc_Pin GPIO_PIN_1
#define UsbVcc_GPIO_Port GPIOB
#define SoundB2_Pin GPIO_PIN_8
#define SoundB2_GPIO_Port GPIOA
#define SoundB3_Pin GPIO_PIN_9
#define SoundB3_GPIO_Port GPIOA
#define SoundB4_Pin GPIO_PIN_10
#define SoundB4_GPIO_Port GPIOA
#define SoundBMute_Pin GPIO_PIN_15
#define SoundBMute_GPIO_Port GPIOA
#define IrReceiver_Pin GPIO_PIN_3
#define IrReceiver_GPIO_Port GPIOB
#define SecondPcb_Pin GPIO_PIN_4
#define SecondPcb_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOB
#define Boot_Pin GPIO_PIN_8
#define Boot_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
