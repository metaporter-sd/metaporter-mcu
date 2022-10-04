/*
 * timers.c
 *
 *  Created on: Sep 30, 2022
 *  Author: Kris Kunovski
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : timers.c
  * @brief          : functions to use timers
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
#include "timers.h"
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

void tim6_init(void) {
  TIM6->CR1 &= ~TIM_CR1_CEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
  TIM6->PSC = 4800-1;
  TIM6->ARR = 10-1;
  TIM6->DIER |= TIM_DIER_UIE;
  NVIC->ISER[0] = 1<<TIM6_DAC_IRQn;
}

void tim6_start(void) {
	TIM6->CR1 |= TIM_CR1_CEN;			// enable timer clock
}

void tim6_stop(void) {
	TIM6->CR1 &= ~TIM_CR1_CEN;			// disable timer clock
}

void tim7_init(void) {
  TIM7->CR1 &= ~TIM_CR1_CEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;	// enable timer 7
  TIM7->PSC = 48000-1;				// Look below for timer speed
  TIM7->ARR = 10-1;					// 48000000 / 48000 / 10 = 100 Hz
  TIM7->DIER |= TIM_DIER_UIE;			// enable update on interrupt
  NVIC->ISER[0] = 1<<TIM7_IRQn;		// enable interrupt handler

}

void tim7_start(void) {
	TIM7->CR1 |= TIM_CR1_CEN;			// enable timer clock
}

void tim7_stop(void) {
	TIM7->CR1 &= ~TIM_CR1_CEN;			// disable timer clock
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
