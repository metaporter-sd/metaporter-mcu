/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : keypad.h
  * @brief          : Header for keypad.c file.
  *
  ******************************************************************************
**/

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __KEYPAD_H
#define __KEYPAD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* User Define */
#define KEYPAD_ROW_SIZE 4			// number of rows
#define KEYPAD_COL_SIZE 4			// number of columns

#define ESCAPE_KEY 'z'

// The keypad matrix mapping
static const char KeyPadMatrix[KEYPAD_ROW_SIZE][KEYPAD_COL_SIZE] = {
		{'D','#','0','*'},
		{'C','9','8','7'},
		{'B','6','5','4'},
		{'A','3','2','1'}
};


/* USER CODE BEGIN EFP */
void keypad_init(void);
void set_row(char offset);
int get_cols(void);
void get_key(char offset, int cols);


/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif
