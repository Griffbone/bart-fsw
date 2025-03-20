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
#include "stm32h7xx_hal.h"

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
#define SPI4_CS_Pin GPIO_PIN_4
#define SPI4_CS_GPIO_Port GPIOE
#define SPI1_CS1_Pin GPIO_PIN_1
#define SPI1_CS1_GPIO_Port GPIOC
#define ACC_INT1_Pin GPIO_PIN_1
#define ACC_INT1_GPIO_Port GPIOA
#define ACC_INT2_Pin GPIO_PIN_3
#define ACC_INT2_GPIO_Port GPIOA
#define SPI2_CS2_Pin GPIO_PIN_4
#define SPI2_CS2_GPIO_Port GPIOC
#define BARO_INT_Pin GPIO_PIN_13
#define BARO_INT_GPIO_Port GPIOE
#define SENSE_1_Pin GPIO_PIN_12
#define SENSE_1_GPIO_Port GPIOB
#define FIRE_1_Pin GPIO_PIN_13
#define FIRE_1_GPIO_Port GPIOB
#define FIRE_2_Pin GPIO_PIN_14
#define FIRE_2_GPIO_Port GPIOB
#define SENSE2_Pin GPIO_PIN_15
#define SENSE2_GPIO_Port GPIOB
#define SENSE_3_Pin GPIO_PIN_8
#define SENSE_3_GPIO_Port GPIOD
#define FIRE_3_Pin GPIO_PIN_9
#define FIRE_3_GPIO_Port GPIOD
#define FIRE_4_Pin GPIO_PIN_10
#define FIRE_4_GPIO_Port GPIOD
#define IMU_INT2_Pin GPIO_PIN_11
#define IMU_INT2_GPIO_Port GPIOD
#define SENSE_4_Pin GPIO_PIN_12
#define SENSE_4_GPIO_Port GPIOD
#define SENSE_5_Pin GPIO_PIN_13
#define SENSE_5_GPIO_Port GPIOD
#define USR_LED_2_Pin GPIO_PIN_14
#define USR_LED_2_GPIO_Port GPIOD
#define FIRE_5_Pin GPIO_PIN_15
#define FIRE_5_GPIO_Port GPIOD
#define FIRE_6_Pin GPIO_PIN_6
#define FIRE_6_GPIO_Port GPIOC
#define USR_LED_1_Pin GPIO_PIN_7
#define USR_LED_1_GPIO_Port GPIOC
#define SENSE_6_Pin GPIO_PIN_9
#define SENSE_6_GPIO_Port GPIOC
#define IMU_INT1_Pin GPIO_PIN_12
#define IMU_INT1_GPIO_Port GPIOA
#define SPI3_CS1_Pin GPIO_PIN_4
#define SPI3_CS1_GPIO_Port GPIOD
#define SPI3_CS2_Pin GPIO_PIN_4
#define SPI3_CS2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
