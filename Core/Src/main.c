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
#include "uart.h"
#include "utilities.h"
//#include "lcd.h"
//#include "keypad.h"
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
int time_remaining = 0;
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
  nano_wait(1000000000);
  //nano_wait(1000000000);
  //nano_wait(1000000000);

  IMU imu;

  imu_init(&imu, IMU_ADDR, IMU_MODE_NDOF);

//  uart3_init();
  //lidar_init();
  //LCD_Setup();
  //Keypad_Init();

  //uart3_test();

  //init_spi2();
  //spi2_init_oled();
  //LCD_DrawString(80, 125, BLACK, WHITE,  ("Metaporter"), 16, 0);
  //LCD_DrawString(80, 145, BLACK, WHITE,  ("Time: 0s"), 16, 0);

  //lidar_test_start_stop(); // passes. scope verified
  //lidar_test_send_one(); // passes. scope verified
  //lidar_test_send(); // passes. scope verified
  //lidar_test_read_one();  // passes. scope verified
  //lidar_wait_for_data(); // passes. scope verified
  //lidar_test_get_one_distance(); // passes. scope verified. Reading takes a long time to be ready
  /*
#define LIDAR_BUFFER_SIZE 200
  uart3_test();

  // UART buffer for lidar data
  uint8_t header[2];
  uart3_create_header(header, UART_COM_NONE, UART_DATA_SOURCE_LIDAR, UART_UINT16_T, LIDAR_BUFFER_SIZE);

  char dist_string[8];

  // send header as string
  sprintf(dist_string, "%d", header[0]);
  uart3_send_string(dist_string);
  uart3_send_string("\n\r");

  sprintf(dist_string, "%d", header[1]);
  uart3_send_string(dist_string);
  uart3_send_string("\n\r");

  // read lidar data into buffer and send as string
  uint16_t dist[LIDAR_BUFFER_SIZE];
  for (int i = 0; i < LIDAR_BUFFER_SIZE; i++) {
	  lidar_get_distance(&dist[i]);

	  // send over uart as string
	  sprintf(dist_string, "%d", dist[i]);
	  uart3_send_string(dist_string);
	  uart3_send_string("\n\r");
  }
  */
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

/* USER CODE BEGIN 4 */
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
//void TIM6_DAC_IRQHandler(void) {
//	char stringy[20];
//    TIM6->SR &= ~TIM_SR_UIF;
//    time_remaining+=1;
//    sprintf(stringy, "Time: %ds", time_remaining);
//    //sprintf(score_string, "Level: %d", level_score);
//    LCD_DrawString(80, 145, BLACK, WHITE,  (stringy), 16, 0);
//
//    spi2_display2(stringy);
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
