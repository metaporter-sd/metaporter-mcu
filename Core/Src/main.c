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
#include <stdio.h>

#include "main.h"
#include "utilities.h"
#include "timers.h"
#include "keypad.h"
#include "uart.h"
#include "dma.h"
#include "lcd.h"
#include "imu.h"

#include "test.h"

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
char offset = 0; // used to scan keys

int time_elapsed = 0;

IMU imu;

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
  //keypad_init();
  uart3_init();
  dma1_init();

  lcd_setup();
  lcd_set_home_screen();


  nano_wait(1000000000);
  imu_init(&imu, IMU_ADDR, IMU_MODE_NDOF);
  tim6_init();
  tim7_init();
  keypad_init();

  tim6_start(); // start timer 6 to scan keypad rows

  //init_exti_pa0(); // for testing
  //init_exti_pb2(); // for testing

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  }
	  /* USER CODE BEGIN 3 */
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
void TIM6_DAC_IRQHandler(void) {
    int cols = get_cols();
    char key_pressed = get_key(offset, cols);
    offset = (offset + 1) & 0x7; // count 0 ... 7 and repeat
    set_row(offset);

    TIM6->SR &= ~TIM_SR_UIF; // clear interrupt

    // set mode based on key pressed
    if (key_pressed == MODE_DATA_COL) {
        start_data_collection(); // start to start is safe
    } else if (key_pressed == MODE_STOP_DATA_COL) {
    	stop_data_collection();
    } else if (key_pressed == MODE_CALIBRATE) {
    	// start to calibrate needs to stop tim7 which is done inside calibrate...safe
    	dummy_calibrate();
    }
}

void DMA1_Ch4_7_DMA2_Ch3_5_IRQHandler(void) {
	if ( DMA1->ISR && DMA_ISR_TCIF7 ) {
		dma_transfers_completed++;
		DMA1->IFCR |= DMA_IFCR_CTCIF7;
	}
}

void TIM7_IRQHandler(void) { // TODO: discard last few readings
    if (count >= 100) { // every 1 second update display with time elapsed
  	    time_elapsed += 1;
  	    lcd_show_elapsed_time(time_elapsed);
        count = 0;
    }


    int timeout = 7; // max 10, 5 = timeout after 5ms
    for (int i = 0; i < timeout; i++) {
		if (dma_transfers_started == dma_transfers_completed) {
			break;
		} //else {
			//char data_string[100];
			//sprintf(data_string, "started: %d, finished: %d\n\r", dma_transfers_started, dma_transfers_completed);
			//uart3_send_string(data_string);
			//tim7_stop();
			//break;
		// }
		nano_wait(1000000); // 10 ms = 10^7 ns,
	}

    imu_get_quat(&imu);
    count++;

    char data_string[100];
	// send data as string
    sprintf(data_string, "(%d, %d, %d, %d)\n\r", imu.quat[0], imu.quat[1], imu.quat[2], imu.quat[3]);
    dma1_start(data_string, (uint32_t) &(USART3->TDR), strlen(data_string));

  // dma1_start(imu.quat, &(USART3->TDR), sizeof(imu.quat));	// sends imu data as bytes
    dma_transfers_started++;

	TIM7->SR &= ~TIM_SR_UIF;
}


void start_data_collection(void) {
	count = 0;
	dma_transfers_started = 0;
	dma_transfers_completed = 0;
	time_elapsed = 0;
	lcd_set_home_screen();
	lcd_update_status("Collecting Data");
	lcd_show_elapsed_time(time_elapsed);
	tim7_start(); // starting timer 7 begins IMU data collection
}

void stop_data_collection (void) { //

	tim7_stop(); // starting timer 7 begins IMU data collection
	//while (dma_transfers_started >= dma_transfers_completed);
//	char data_string[100];
//	sprintf(data_string, "started: %d, finished: %d\n\r", dma_transfers_started, dma_transfers_completed);
//	uart3_send_string(data_string);

	// wait for last transfer to finish before moving on
//	while (dma_transfers_started != dma_transfers_completed) {
//		nano_wait(100000); // 10 ms = 10^7 ns,
//	}

	lcd_set_home_screen();
	lcd_update_status("Finished");
}

void dummy_calibrate (void) {
	tim7_stop(); // starting timer 7 begins IMU data collection
	lcd_set_home_screen();
	lcd_update_status("Calibrating");

	// TODO: add actual calibration
	for (int i = 0; i < 2; i++) {
		nano_wait(1000000000);
	}

	lcd_update_status("Finished calibration");

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
