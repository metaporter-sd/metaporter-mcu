/*
 * test.c
 *
 *  Created on: Sep 26, 2022
 *  Author: Jehan Shah
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : test.c
  * @brief          : functions used to initialize subsystems used for testing
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
#include "test.h"
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

void init_exti_pa0(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	SYSCFG->EXTICR[0] &= ~(0x7); // set the source for pa
	EXTI -> IMR |= EXTI_IMR_MR0;
	EXTI->RTSR |= EXTI_RTSR_TR0;
	NVIC->ISER[0] = 1<<EXTI0_1_IRQn;

}

void init_exti_pb2(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;			// enable GPIO pin B
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR2);		// clear pull-up/pull-down reg
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR2_1;		// set pupdr to pull-down for GPIO pin B
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;	// SYSCFG clock enable
	SYSCFG->EXTICR[0] |= 1<<(4 * 2);			// set source input to PB pins for interrupt
	EXTI->IMR |= EXTI_IMR_MR2;					// unmask interrupt request for EXTI Line 2
	EXTI->RTSR |= EXTI_RTSR_TR2;				// enable rising trigger for EXTI Line 2
	NVIC->ISER[0] = 1<<EXTI2_3_IRQn;			// acknowledge and enable EXTI interrupt
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


