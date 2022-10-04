/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : keypad.c
  * @brief          : Keypad initialization and helper functions
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
#include "new_keypad.h"
#include "uart.h"
#include "utilities.h"
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
char history[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */




/* USER CODE BEGIN 4 */

void keypad_init(void) {

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;								// Enable GPIOA
	GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 |		// Clear MODER bits for pins 0-7
					  GPIO_MODER_MODER2 | GPIO_MODER_MODER3 |		// cols
					  GPIO_MODER_MODER4 | GPIO_MODER_MODER5 |
					  GPIO_MODER_MODER6 | GPIO_MODER_MODER7);		// rows
	GPIOA->MODER |= GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 |
					GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0;		// Set output mode for PA4-PA7
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 |
					  GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3);		// Clear PUPDR for inputs
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR1_1 |
					GPIO_PUPDR_PUPDR2_1 | GPIO_PUPDR_PUPDR3_1;		// Set inputs to pull-down
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5 |
					  GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);		// Clear PUPDR for outputs
//	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1 | GPIO_PUPDR_PUPDR5_1 |
//					GPIO_PUPDR_PUPDR6_1 | GPIO_PUPDR_PUPDR7_1;		// Set outputs to pull-down
}


void set_row(char offset) {
    int row = (offset & 3) + 4;
    GPIOA->BSRR = 0xf00000 | (1<<row); // Set the row active on the keypad matrix. (reset others)
}


int get_cols(void) {
    return (GPIOA->IDR) & 0xf; // Read the column pins of the keypad matrix. (PA[3:0])
}


// Check the columns for a row of the keypad and update state.
// If a col is updated to 0x01, update state.
void update_state(char offset, int cols) {
    int row = offset & 3;
    char key_pressed = 'z';
    for(int i=0; i < 4; i++) {
        history[4*row+i] = (history[4*row+i]<<1) + ((cols>>i)&1);
        if (history[4*row+i] == 0x1) {

        	key_pressed = KeyPadMatrix[row][i];
        }
    }
    if(key_pressed != 'z') {
		uart3_send_byte(key_pressed);
	}

}


void TIM6_DAC_IRQHandler(void) {
    int cols = get_cols();
    update_hist(cols);
    offset = (offset + 1) & 0x7; // count 0 ... 7 and repeat
    set_row();
    TIM6->SR &= ~TIM_SR_UIF;
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
