/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart.c
  * @brief          : UART initialization and data transfer
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
#include "uart.h"
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

/* USER CODE END 0 */




/* USER CODE BEGIN 4 */

void GPIO_Init(void) {

	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;			// Enable GPIOC
	GPIOC->MODER &= ~GPIO_MODER_MODER10;		// Clear GPIOC MODER10 bits
	GPIOC->MODER |= GPIO_MODER_MODER10_1;		// Set alternate function bits
	GPIOC->MODER &= ~GPIO_MODER_MODER11;		// Clear GPIOC MODER11 bits
	GPIOC->MODER |= GPIO_MODER_MODER11_1;		// Set alternate function bits
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR10;	// Set high speed output for GPIOC pin 10
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR11;	// Set high speed output for GPIOC pin 11
	GPIOC->AFR[1]  |= (1<<8);					// Set AF1 for GPIOC pin 10
	GPIOC->AFR[1]  |= (1<<12);					// Set AF1 for GPIOC pin 11

}

void USART3_UART_Init(void) {

	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;	// Enable USART3
	USART3->CR1 &= ~USART_CR1_UE;			// Disable UE (USART3)
	USART3->CR1 &= ~(0x3<<28);				// Set word length (M0) to 1 Start bit, 8 data bits, n stop bits
	USART3->CR2 &= ~(0x3<<12);				// Set stop bit to 1
	USART3->CR1 &= ~USART_CR1_PCE;			// Disable parity control
	USART3->CR1 &= ~USART_CR1_OVER8;		// Set oversampling by 16
	USART3->BRR = 0x1a1;					// Set baud rate to 115200 bits/s (0x1a1 = 417 = 48000000 / 115200)
	USART3->CR1 |= 1<<2;					// Receiver is enabled
	USART3->CR1 |= 1<<3;					// Transmitter is enabled
	USART3->CR1 |= 1;						// Enable UE (USART3)

	while(((USART3->ISR & USART_ISR_REACK) != USART_ISR_REACK) && ((USART3->ISR & USART_ISR_TEACK) != USART_ISR_TEACK));

}

void USART3_DMA1_Init(const short * data) {

    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;					// Make sure DMA is off
    DMA1_Channel7->CPAR = (uint32_t)(&(USART3->TDR));	// Copy to address in CPAR (USART3 TX)
    DMA1_Channel7->CMAR = (uint32_t)(data);				// Copy from address in CMAR
    DMA1_Channel7->CNDTR = 34;							// Copy this many data
    DMA1_Channel7->CCR |= DMA_CCR_DIR;					// Read from "memory"
    DMA1_Channel7->CCR |= DMA_CCR_MINC;					// Increment CMAR as we copy
    DMA1_Channel7->CCR &= ~(DMA_CCR_PSIZE);				// 00: 8 bits
    DMA1_Channel7->CCR &= ~(DMA_CCR_MSIZE);				// 00: 8 bits
    //DMA1_Channel7->CCR |= DMA_CCR_CIRC;					// Enable circular buffer
    NVIC->ISER[0] = 1<<DMA1_Channel7_IRQn;				// Enable the interrupt

}

void enable_dma(void) {

    DMA1_Channel7->CCR |= 1;	// Enable DMA

}

void transmitChar(uint8_t c) {

	while((USART3->ISR & USART_ISR_TXE) != USART_ISR_TXE);
	USART3->TDR = c;
	while((USART3->ISR & USART_ISR_TXE) != USART_ISR_TXE);

}

void transmitString(char * str) {

	while(*str) {
		transmitChar(*str++);
	}

}

/* USER CODE END 4 */


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
