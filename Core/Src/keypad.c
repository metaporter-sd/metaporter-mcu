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
	//enable EXTI interrupts pin setup


	// Setting up row for output high
	for (int idx = 0; idx < KEYPAD_ROW_SIZE; idx++)
	{
		GPIO_InitTypeDef RowDefault =
		{
				.Pin = kpPinout.Rows[idx].GPIO_Pin,
				.Mode = GPIO_MODE_OUTPUT_PP,
				.Pull = GPIO_PULLDOWN,
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
					.Pull = GPIO_NOPULL,
					.Speed = GPIO_SPEED_FREQ_LOW,
					.Alternate = 0
			};
			HAL_GPIO_Init(kpPinout.Cols[idx].GPIOx,&ColDefault);

			//HAL_GPIO_WritePin(kpPinout.Cols[idx].GPIOx,kpPinout.Cols[idx].GPIO_Pin,GPIO_PIN_RESET);
		}

	init_exti();
}

void init_exti(void) {
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;										// SYSCFG clock enable
	SYSCFG->EXTICR[2] &= 0;															// set source input to PA pins for interrupt
	EXTI->IMR |= EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR6 | EXTI_IMR_MR7;			// unmask interrupt request for EXTI Lines 4-7
	EXTI->RTSR |= EXTI_RTSR_TR4 | EXTI_RTSR_TR5 | EXTI_RTSR_TR6 | EXTI_RTSR_TR7;	// enable rising trigger for EXTI Lines 4-7
	NVIC->ISER[0] = 1<<EXTI4_15_IRQn;												// acknowledge and enable EXTI interrupt
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
	for (int idx; idx < KEYPAD_ROW_SIZE; idx++)
	{
		HAL_GPIO_WritePin(kpPinout.Rows[idx].GPIOx,kpPinout.Rows[idx].GPIO_Pin,GPIO_PIN_SET);
	}

	return retVal;
}





