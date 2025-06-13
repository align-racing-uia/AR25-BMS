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
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define hqspi hqspi1
#include "math.h" // Needed for the w25q_mem.h
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
void USB_CDC_RxHandler(uint8_t*, uint32_t);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Ext_WD_Sig_Pin GPIO_PIN_13
#define Ext_WD_Sig_GPIO_Port GPIOC
#define Low_Current_Sensor_Pin GPIO_PIN_0
#define Low_Current_Sensor_GPIO_Port GPIOA
#define AMS_Fault_Pin GPIO_PIN_1
#define AMS_Fault_GPIO_Port GPIOA
#define SDC_Pin GPIO_PIN_4
#define SDC_GPIO_Port GPIOA
#define High_Current_Sensor_Pin GPIO_PIN_5
#define High_Current_Sensor_GPIO_Port GPIOA
#define Temperature_Pin GPIO_PIN_2
#define Temperature_GPIO_Port GPIOB
#define SPIRDY_Pin GPIO_PIN_11
#define SPIRDY_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define nFault_Pin GPIO_PIN_8
#define nFault_GPIO_Port GPIOA
#define Precharge_Pin GPIO_PIN_3
#define Precharge_GPIO_Port GPIOB
#define Minus_Pin GPIO_PIN_4
#define Minus_GPIO_Port GPIOB
#define Plus_Pin GPIO_PIN_5
#define Plus_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define ALIGN_CAN_USE_BUFFER // Enable the CAN buffer for the FDCAN peripheral

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
