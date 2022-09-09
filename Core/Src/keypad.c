#include "keypad.h"
#include <assert.h>

static int Keypad_Search_Row(int colIdx);

// Initalization of Keypad using HAL
// Keypad will be configured in such a way that
// Row will be the output and the Column will be the input
// @Param = None
// @ReturnVal = None
void Keypad_Init()
{

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// Setting up row for output high
	for (int idx = 0; idx < KEYPAD_ROW_SIZE; idx++)
	{
		GPIO_InitTypeDef RowDefault =
		{
				.Pin = KeypadRowPins[idx].GPIO_Pin,
				.Mode = GPIO_MODE_OUTPUT_PP,
				.Pull = GPIO_PULLDOWN,
				.Speed = GPIO_SPEED_FREQ_LOW,
				.Alternate = 0
		};
		HAL_GPIO_Init(KeypadRowPins[idx].GPIOx,&RowDefault);

		// Toggles to high for output
		HAL_GPIO_WritePin(KeypadRowPins[idx].GPIOx,KeypadRowPins[idx].GPIO_Pin,GPIO_PIN_SET);


	}

	// Setting up col for input
	for (int idx = 0; idx < KEYPAD_COL_SIZE; idx++)
	{
			GPIO_InitTypeDef ColDefault =
			{
					.Pin = KeypadColPins[idx].GPIO_Pin,
					.Mode = GPIO_MODE_INPUT,
					.Pull = GPIO_NOPULL,
					.Speed = GPIO_SPEED_FREQ_LOW,
					.Alternate = 0
			};
			HAL_GPIO_Init(KeypadColPins[idx].GPIOx,&ColDefault);

			//HAL_GPIO_WritePin(KeypadColPins[idx].GPIOx,KeypadColPins[idx].GPIO_Pin,GPIO_PIN_RESET);
		}
}

// Scans the keypad to find any outputs
// This is the function to invoke in a IRQ or Timer routine (TBD)
// @Param returnChar = returns the keypad option if keypad is pressed
// @ReturnVal = the status of keypad, 0 for not pressed and 1 for pressed
int Keypad_Scan(char* returnChar)
{
	int KeyNotPressed = 0;
	int KeyPressed = 1;

	for (int colIdx = 0; colIdx < KEYPAD_COL_SIZE; colIdx++)
	{
		// If the column pin input is pulled high (registered press)
		if (HAL_GPIO_ReadPin(KeypadColPins[colIdx].GPIOx,KeypadColPins[colIdx].GPIO_Pin))
		{
			int rowIdx = Keypad_Search_Row(colIdx);
			assert(rowIdx != -1);
			*returnChar = KeyPadMatrix[rowIdx][colIdx];
			return KeyPressed;
		}

	}

	return KeyNotPressed;
}

// Toggles the output row signal to find the pressed button
// Helper function of Keypad_Scan
// toggleConfig goes from 1000, 0100, 0010, 0001 to test out the button read
// @Param colIdx = column index that registered a button pressed
// @ReturnVal = the row index that registered a button pressed , -1 for state of error
int Keypad_Search_Row(int colIdx)
{
	int retVal = -1;

	// Column pin config goes from 1000, 0100, 0010, to 0001
	for (int toggleConfig = 0; toggleConfig < KEYPAD_ROW_SIZE; toggleConfig++)
	{
		// Going through each row pins to set the output signal logic
		for(int idx = 0; idx < KEYPAD_ROW_SIZE; idx++)
		{
			uint8_t BitState = GPIO_PIN_RESET; // 0u
			if(idx == toggleConfig) BitState = GPIO_PIN_SET; // 1u

			HAL_GPIO_WritePin(KeypadRowPins[idx].GPIOx,KeypadRowPins[idx].GPIO_Pin,BitState);

		}

		// If the column pin input is still registered as high, (found the corresponding row pin)
		if (HAL_GPIO_ReadPin(KeypadColPins[colIdx].GPIOx,KeypadColPins[colIdx].GPIO_Pin))
		{
			retVal = toggleConfig;
			break;
		}
	}

	//Reset all row output pins to high
	for (int idx; idx < KEYPAD_ROW_SIZE; idx++)
	{
		HAL_GPIO_WritePin(KeypadRowPins[idx].GPIOx,KeypadRowPins[idx].GPIO_Pin,GPIO_PIN_SET);
	}

	return retVal;
}





