/*
 * dma.c
 *
 *  Created on: Sep 26, 2022
 *  Author: Jehan Shah
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : dma.c
  * @brief          : functions used for dma transfer
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
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dma.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void dma1_init(void) {

    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;					// Make sure DMA is off
    DMA1_Channel7->CCR |= DMA_CCR_DIR;					// Read from "memory"
    DMA1_Channel7->CCR |= DMA_CCR_MINC;					// Increment CMAR as we copy
    DMA1_Channel7->CCR &= ~(DMA_CCR_PSIZE);				// 00: 8 bits
    DMA1_Channel7->CCR &= ~(DMA_CCR_MSIZE);				// 00: 8 bits
//    DMA1_Channel7->CCR |= DMA_CCR_CIRC;					// Enable circular buffer
//    DMA1_Channel7->CCR |= DMA_CCR_TCIE;					// Enable transfer complete interrupt
//    NVIC->ISER[0] = 1<<DMA1_Channel4_5_6_7_IRQn;		// Enable the interrupt

}

void dma1_start(void * src, uint32_t dst, uint16_t num_bytes) {
	DMA1_Channel7->CCR &= ~1;				// disable DMA
    DMA1_Channel7->CMAR = (uint32_t)(src);	// Copy from address in CMAR
    DMA1_Channel7->CPAR = (uint32_t)(dst);	// Copy to address in CPAR (USART3 TX)
    DMA1_Channel7->CNDTR = num_bytes;		// Copy this many data
    DMA1_Channel7->CCR |= 1;				// Enable DMA

}

/* USER CODE END 0 */


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
