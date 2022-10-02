#include "stm32f0xx.h"
#include "lcd.h"
#include <string.h>

// Be sure to change this to your login...

int time_remaining = 0;
int DATA_COLLECTION_GLOBAL_FLAG = 0;


void set_char_msg(int, char);
void nano_wait(unsigned int);
void LCD_update_status(char *);




int msg_index = 0;
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];



void small_delay(void) {
    nano_wait(50000000);
}







void TIM6_DAC_IRQHandler(void) {
	char stringy[100];
    TIM6->SR &= ~TIM_SR_UIF;


	time_remaining+=1;
	//sprintf(stringy, "MetaPorter has been capturing data for: %ds", time_remaining);
	LCD_DrawString(0, 240, BLACK, WHITE, ("Metaporter has been capturing"), 16, 0);
	sprintf(stringy, "data for: %ds", time_remaining); //convert int time into string
	LCD_DrawString(0, 275, BLACK, WHITE,  (stringy), 16, 0);



}


void init_exti_pb2(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;			// enable GPIO pin B
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR2);		// clear pull-up/pull-down reg
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR2_1;		// set pupdr to pull-down for GPIO pin B
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;	// SYSCFG clock enable
	SYSCFG->EXTICR[0] |= 1<<(4 * 2);			// set source input to PB pins for interrupt
	EXTI->IMR |= EXTI_IMR_MR2;					// unmask interrupt request for EXTI Line 2
	EXTI->RTSR |= EXTI_RTSR_TR2;				// enable rising trigger for EXTI Line 2
	NVIC->ISER[0] = 1<<EXTI2_3_IRQn;			// acknowledge and enable EXTI interrupt
}

void init_exti_pa0(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	SYSCFG->EXTICR[0] &= ~(0x7); // set the source for pa
	EXTI -> IMR |= EXTI_IMR_MR0;
	EXTI->RTSR |= EXTI_RTSR_TR0;
	NVIC->ISER[0] = 1<<EXTI0_1_IRQn;

}

void EXTI0_1_IRQHandler(void) {
	time_remaining = 0;
	char init_status[] = "Data Collection";
	LCD_update_status(init_status);
	tim6_start();

	EXTI->PR |= EXTI_PR_PR0;

}

void EXTI2_3_IRQHandler(void) {
	char status[] = "Finished";

	tim6_stop();
	LCD_update_status(status);
	EXTI->PR |= EXTI_PR_PR2;
}

void tim6_init(void) {
  TIM6->CR1 &= ~TIM_CR1_CEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
  TIM6->PSC = 48000-1;
  TIM6->ARR = 1000-1;
  TIM6->DIER |= TIM_DIER_UIE;
  NVIC->ISER[0] = 1<<TIM6_DAC_IRQn;
}

void tim6_start(void) {
	TIM6->CR1 |= TIM_CR1_CEN;			// enable timer clock
}

void tim6_stop(void) {
	TIM6->CR1 &= ~TIM_CR1_CEN;			// disable timer clock
}

void LCD_update_status(char * status) {

	LCD_Clear(0xffffff);
	LCD_DrawString(5, 100, BLACK, WHITE, "Status: ", 16, 0);
	LCD_DrawString(70, 100, BLACK, WHITE, (status), 16, 0);

}

int main(void)
{

    // This time, autotest always runs as an invisible aid to you.
    //autotest();

    // GPIO enable
    //enable_ports();
    SystemClock_Config();
    LCD_Setup();
    init_exti_pb2();
    init_exti_pa0();
    tim6_init();
    //tim6_start();
}
