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
#include "utilities.h"
#include "timers.h"
#include "uart.h"
#include "dma.h"
#include "lcd.h"
#include "imu.h"
#include <stdio.h>


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

/* USER CODE BEGIN PV */
int time_elapsed = 0;

IMU imu;

char dma_test[8] = "hello\n\r"; // to test dma

// to make sure dma transfer is complete before new reading
int count = 0;
int dma_transfers_started = 0;
int dma_transfers_completed = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  keypad_init();
  uart3_init();
  dma1_init();
  nano_wait(1000000000);
  imu_init(&imu, IMU_ADDR, IMU_MODE_NDOF);
  tim6_init();
  tim7_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//	  	char data_string[64];
//
//		imu_get_quat(&imu);
//		// send data as string
//		sprintf(data_string, "(%d, %d, %d, %d)\n\r", imu.x, imu.y, imu.z, imu.w);
//		uart3_send_string(data_string);
//		nano_wait(100000000);

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

/* USER CODE BEGIN 4 */

// Interrupt Service Routines 

void EXTI0_1_IRQHandler(void) { // TODO: copy this over to keypad based interrupt where status = data_col
	time_elapsed = 0; 
	lcd_update_status("collecting data");

  tim6_start(); // start timer 6 to display time to screen
  tim7_start(); // starting timer 7 begins IMU data collection
	
	EXTI->PR |= EXTI_PR_PR0;
}

void EXTI2_3_IRQHandler(void) { // TODO: copy this over to keypad based interrupt where mode = st
	tim6_stop(); // start timer 6 to display time to screen
  tim7_stop(); // starting timer 7 begins IMU data collection

	lcd_update_status("finished collecting data");

	EXTI->PR |= EXTI_PR_PR2;
}

void TIM6_DAC_IRQHandler(void) {
	char stringy[100];
  TIM6->SR &= ~TIM_SR_UIF;

	time_elapsed += 1;
	//sprintf(stringy, "MetaPorter has been capturing data for: %ds", time_elapsed);
	LCD_DrawString(0, 240, BLACK, WHITE, ("Metaporter has been capturing"), 16, 0);
	sprintf(stringy, "data for: %ds", time_elapsed); //convert int time into string
	LCD_DrawString(0, 275, BLACK, WHITE,  (stringy), 16, 0);
}

void TIM7_IRQHandler(void) {
//	DMA1->IFCR |= DMA_IFCR_CGIF7;
// int timeout = 8000; // times out after 5ms

	if ( count < 50 ) {
//		for (int i = 0; i < timeout; i++) {
//			if (dma_transfers_started == dma_transfers_completed) {
//				break;
//			}
//			nano_wait(1000);
//		}
		imu_get_quat(&imu);

		char data_string[100];
		// send data as string
		sprintf(data_string, "(%d, %d, %d, %d)\n\r", imu.quat[0], imu.quat[1], imu.quat[2], imu.quat[3]);
		dma1_start(data_string, (uint32_t) &(USART3->TDR), strlen(data_string));

    // dma1_start(imu.quat, &(USART3->TDR), sizeof(imu.quat));	// sends imu data as bytes
		
    // dma_transfers_started++;
	}
	count++;

	TIM7->SR &= ~TIM_SR_UIF;
}


//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == GPIO_PIN_8){
//		char buttonVal;
//		if(Keypad_Scan(&buttonVal))
//		{
//			uart3_send_byte(buttonVal);
//		}
//	}
//}
//
//void EXTI4_15_IRQHandler(void)
//{
//	HAL_GPIO_EXTI_Callback(GPIO_PIN_8);
//
//}
//
//void DMA1_Ch4_7_DMA2_Ch3_5_IRQHandler(void) {
//	if ( DMA1->ISR && DMA_ISR_TCIF7 ) {
//		DMA1->IFCR |= DMA_IFCR_CTCIF7;
//		dma_transfers_completed++;
//	}
//}


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
