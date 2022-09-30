/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : dma.h
  * @brief          : Header for dma.
  *
  ******************************************************************************
**/

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H
#define __DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

void dma1_init(void);

void dma1_start(void * src, void * dst, uint16_t num_bytes);


/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif
