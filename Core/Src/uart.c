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

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;			// Enable GPIOB
	GPIOB->MODER &= ~GPIO_MODER_MODER3;			// Clear GPIOB MODER3 bits
	GPIOB->MODER |= GPIO_MODER_MODER3_1;		// Set alternate function bits
	GPIOB->MODER &= ~GPIO_MODER_MODER4;			// Clear GPIOB MODER4 bits
	GPIOB->MODER |= GPIO_MODER_MODER4_1;		// Set alternate function bits
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR3;	// Set high speed output for GPIOB pin 3
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR4;	// Set high speed output for GPIOB pin 4
	GPIOB->AFR[0]  |= (4<<12);					// Set AF4 for GPIOB pin 3
	GPIOB->AFR[0]  |= (4<<16);					// Set AF4 for GPIOB pin 4

}

void USART5_UART_Init(void) {

	RCC->APB1ENR |= RCC_APB1ENR_USART5EN;	// Enable USART5
	USART5->CR1 &= ~USART_CR1_UE;			// Disable UE (USART5)
	USART5->CR1 &= ~(0x3<<28);				// Set word length (M0) to 1 Start bit, 8 data bits, n stop bits
	USART5->CR2 &= ~(0x3<<12);				// Set stop bit to 1
	USART5->CR1 &= ~USART_CR1_PCE;			// Disable parity control
	USART5->CR1 &= ~USART_CR1_OVER8;		// Set oversampling by 16
	USART5->BRR = 0x1a1;					// Set baud rate to 115200 bits/s (0x1a1 = 417 = 48000000 / 115200)
	USART5->CR1 |= 1<<2;					// Receiver is enabled
	USART5->CR1 |= 1<<3;					// Transmitter is enabled
	USART5->CR1 |= 1;						// Enable UE (USART5)

	while(((USART5->ISR & USART_ISR_REACK) != USART_ISR_REACK) && ((USART5->ISR & USART_ISR_TEACK) != USART_ISR_TEACK));

}

void transmitChar(uint8_t c) {

	while((USART5->ISR & USART_ISR_TXE) != USART_ISR_TXE);
	USART5->TDR = c;
	while((USART5->ISR & USART_ISR_TXE) != USART_ISR_TXE);

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
