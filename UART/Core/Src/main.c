/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
uint8_t txdata[30] = "Hi everyone!\n\r";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART5_UART_Init(void);
/* USER CODE BEGIN PFP */
static void GPIO_Init(void);
static void USART5_UART_Init(void);
void transmitChar(uint8_t);
void transmitString(char *);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  GPIO_Init();

  USART5_UART_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_USART5_UART_Init();
  /* USER CODE BEGIN 2 */
//  HAL_UART_Transmit(&huart5, txdata, sizeof(txdata), 100);
//  HAL_UART_Transmit(&huart5, "zdrav\n\r", sizeof("zdrav\n\r"), 100);
//  HAL_UART_Transmit(&huart5, "ne\n\r", sizeof("ne\n\r"), 100);

  transmitString("Hello\n\r");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

static void GPIO_Init(void) {

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

static void USART5_UART_Init(void) {

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

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

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
