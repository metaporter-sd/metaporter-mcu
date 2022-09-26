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

typedef struct
{
	GPIO_TypeDef* GPIOx;	// The port where GPIO is (A..F) utilizing CMSIS, ex: GPIOA
	uint16_t GPIO_Pin;		// The pin number of the GPIO (GPIO_PIN_0..15) utilizing HAL library, ex: GPIO_PIN_O
}GPIO_Pins;

typedef struct
{
	GPIO_Pins Rows[KEYPAD_ROW_SIZE];
	GPIO_Pins Cols[KEYPAD_COL_SIZE];
}GPIO_KeypadPinout;

// The keypad pinout
static const GPIO_KeypadPinout kpPinout = {
		.Rows = {
		{GPIOA,GPIO_PIN_0},
		{GPIOA,GPIO_PIN_1},
		{GPIOA,GPIO_PIN_2},
		{GPIOA,GPIO_PIN_3}
		},

		.Cols = {
		{GPIOA,GPIO_PIN_4},
		{GPIOA,GPIO_PIN_5},
		{GPIOA,GPIO_PIN_6},
		{GPIOA,GPIO_PIN_7}
		}
};

// The keypad matrix mapping
static const char KeyPadMatrix[KEYPAD_ROW_SIZE][KEYPAD_COL_SIZE] = {
		{'1','2','3','A'},
		{'4','5','6','B'},
		{'7','8','9','C'},
		{'*','0','#','D'}
};


/* USER CODE BEGIN EFP */
void Keypad_Init(void);
int Keypad_Scan(char* returnChar);


/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif
