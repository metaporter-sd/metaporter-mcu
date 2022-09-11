/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart.h
  * @brief          : Header for uart.c file.
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
#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f0xx_hal.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define UART_COM_NONE 0b00
#define UART_COM_START_DATA_COLLECTION 0b01
#define UART_COM_STOP_DATA_COLLECTION 0b11

#define UART_DATA_SOURCE_SHIFT 2
#define UART_DATA_SOURCE_LIDAR 0b01 << UART_DATA_SOURCE_SHIFT
#define UART_DATA_SOURCE_IMU 0b10 << UART_DATA_SOURCE_SHIFT

#define UART_DATA_TYPE_SHIFT 4
#define UART_UINT8_T 0b0001 << UART_DATA_TYPE_SHIFT
#define UART_UINT16_T 0b0010 << UART_DATA_TYPE_SHIFT
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */
void uart3_init(void);
void uart3_create_header(uint8_t* pheader, uint8_t command, uint8_t d_source, uint8_t d_type, uint8_t num_data);
void uart3_send_byte(uint8_t);
void uart3_send_string(char *);

void uart3_test(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
void uart3_gpio_init(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */
