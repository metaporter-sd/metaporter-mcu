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
	//Port setup
	//enable GPIOA RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	/* Enable GPIOA clock (same as enabling ahb rcc)*/
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	/* Enable SYSCFG clock */
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);


	// Setting up row for output high
	for (int idx = 0; idx < KEYPAD_ROW_SIZE; idx++)
	{
		GPIO_InitTypeDef RowDefault =
		{
				.Pin = kpPinout.Rows[idx].GPIO_Pin,
				.Mode = GPIO_MODE_OUTPUT_PP,
				.Pull = GPIO_PULLUP,
				.Speed = GPIO_SPEED_FREQ_LOW,
				.Alternate = 0
		};
		HAL_GPIO_Init(kpPinout.Rows[idx].GPIOx,&RowDefault);

		// Toggles to high for output
		HAL_GPIO_WritePin(kpPinout.Rows[idx].GPIOx,kpPinout.Rows[idx].GPIO_Pin,GPIO_PIN_SET);


	}

	// Setting up col for input
	for (int idx = 0; idx < KEYPAD_COL_SIZE; idx++)
	{
			GPIO_InitTypeDef ColDefault =
			{
					.Pin = kpPinout.Cols[idx].GPIO_Pin,
					.Mode = GPIO_MODE_INPUT,
					.Pull = GPIO_PULLDOWN,
					.Speed = GPIO_SPEED_FREQ_LOW,
					.Alternate = 0
			};
			HAL_GPIO_Init(kpPinout.Cols[idx].GPIOx,&ColDefault);

			//HAL_GPIO_WritePin(kpPinout.Cols[idx].GPIOx,kpPinout.Cols[idx].GPIO_Pin,GPIO_PIN_RESET);
	}

	//Setup EXTI interrupts

	EXTI_ConfigTypeDef IrqSettings =
	{
			.Line = EXTI_LINE_8,
			.Mode = EXTI_MODE_INTERRUPT,
			.Trigger = EXTI_TRIGGER_RISING,
			.GPIOSel = EXTI_GPIOA
	};
	GPIO_InitTypeDef ItDefault =
				{
						.Pin = GPIO_PIN_8,
						.Mode = GPIO_MODE_IT_RISING,
						.Pull = GPIO_NOPULL,
						.Speed = GPIO_SPEED_FREQ_LOW,
						.Alternate = 0
				};
	HAL_GPIO_Init(GPIOA,&ItDefault);
	EXTI_HandleTypeDef IrqHandle;
	IrqHandle.Line = IrqSettings.Line;
	HAL_EXTI_SetConfigLine(&IrqHandle,&IrqSettings);
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2U, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
		if (HAL_GPIO_ReadPin(kpPinout.Cols[colIdx].GPIOx,kpPinout.Cols[colIdx].GPIO_Pin))
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

			HAL_GPIO_WritePin(kpPinout.Rows[idx].GPIOx,kpPinout.Rows[idx].GPIO_Pin,BitState);

		}

		// If the column pin input is still registered as high, (found the corresponding row pin)
		if (HAL_GPIO_ReadPin(kpPinout.Cols[colIdx].GPIOx,kpPinout.Cols[colIdx].GPIO_Pin))
		{
			retVal = toggleConfig;
			break;
		}
	}

	//Reset all row output pins to high
	for (int idx = 0; idx < KEYPAD_ROW_SIZE; idx++)
	{
		HAL_GPIO_WritePin(kpPinout.Rows[idx].GPIOx,kpPinout.Rows[idx].GPIO_Pin,GPIO_PIN_SET);
	}

	return retVal;
}





