#include "stm32f0xx.h"
#include "lcd.h"
#include <string.h>

// Be sure to change this to your login...
const char login[] = "mbhasin";
int time_remaining = 0;

void set_char_msg(int, char);
void nano_wait(unsigned int);


//===========================================================================
// Configure GPIOC
//===========================================================================
void enable_ports(void) {
    // Only enable port C for the keypad
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~0xffff;
    GPIOC->MODER |= 0x55 << (4*2);
    GPIOC->OTYPER &= ~0xff;
    GPIOC->OTYPER |= 0xf0;
    GPIOC->PUPDR &= ~0xff;
    GPIOC->PUPDR |= 0x55;
}

uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()

//===========================================================================
// Configure timer 7 to invoke the update interrupt at 1kHz
// Copy from lab 6 or 7.
//===========================================================================
/*void init_tim7() {
    RCC -> APB1ENR |= (1<<5);
    TIM7->PSC = 4800 - 1;

    TIM7->ARR = 10 - 1;

    TIM7 -> DIER |= (1);
    TIM7 -> CR1 |= 0x1;
    NVIC -> ISER[0] = (1 << TIM7_IRQn);

}

//===========================================================================
// Copy the Timer 7 ISR from lab 7
//===========================================================================
void TIM7_IRQHandler (void) {
    TIM7->SR &= ~1;
    int rows = read_rows();
    update_history(col, rows);
    col = (col + 1) & 3;
    drive_column(col);

} */

//===========================================================================
// 2.1 Bit Bang SPI LED Array
//===========================================================================
int msg_index = 0;
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];

//===========================================================================
// Configure PB12 (NSS), PB13 (SCK), and PB15 (MOSI) for outputs
//===========================================================================
void setup_bb(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB->MODER &= ~0xcf000000;
    GPIOB->MODER |= 0x45000000;

    GPIOB -> ODR |= 0x1000;


}

void small_delay(void) {
    nano_wait(50000000);
}

//===========================================================================
// Set the MOSI bit, then set the clock high and low.
// Pause between doing these steps with small_delay().
//===========================================================================
void bb_write_bit(int val)
{
    //int and_val = val & 1;
    if (val != 0) {
        GPIOB -> BSRR = (1 << 15);
    }
    else {
        GPIOB -> BRR = (1<<15);
    }
    small_delay();
    GPIOB -> BSRR = (1 << 13);
    small_delay();
    GPIOB -> BRR = (1 << 13);


    // NSS (PB12)
    // SCK (PB13)
    // MOSI (PB15)

}

//===========================================================================
// Set NSS (PB12) low,
// write 16 bits using bb_write_bit,
// then set NSS high.
//===========================================================================
void bb_write_halfword(int halfword)
{
    GPIOB -> BRR = (1 << 12);
    for (int shift_val = 15; shift_val >= 0; shift_val--) {
            int a = (halfword >> shift_val) & 1;
            bb_write_bit(a);
    }
    GPIOB -> BSRR = (1 << 12);

}

//===========================================================================
// Continually bitbang the msg[] array.
//===========================================================================
void drive_bb(void) {
    for(;;)
        for(int d=0; d<8; d++) {
            bb_write_halfword(msg[d]);
            nano_wait(1000000); // wait 1 ms between digits
        }
}

//============================================================================
// setup_dma()
// Copy this from lab 6 or lab 7.
// Write to SPI2->DR instead of GPIOB->ODR.
//============================================================================
void setup_dma(void)
{
    RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CPAR = (uint32_t) 0x4000380C;
    DMA1_Channel5->CMAR = (uint32_t) msg;
    DMA1_Channel5->CNDTR = 0x8;

    DMA1_Channel5->CCR |= DMA_CCR_DIR;
    DMA1_Channel5->CCR |= DMA_CCR_MINC;
    DMA1_Channel5->CCR |= 0x400;
    DMA1_Channel5->CCR |= 0x100;
    DMA1_Channel5->CCR |= 1<<5;
}

//============================================================================
// enable_dma()
// Copy this from lab 6 or lab 7.
//============================================================================
void enable_dma(void)
{
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}

//============================================================================
// Configure Timer 15 for an update rate of 1 kHz.
// Trigger the DMA channel on each update.
// Copy this from lab 6 or lab 7.
//============================================================================
void init_tim15(void)
{
    RCC->APB2ENR |= (1 << 16);

    TIM15->PSC = 4800 - 1;

    TIM15->ARR = 10 - 1;

    TIM15 -> DIER |= (1 << 8);
    TIM15 -> CR1 |= 0x1;
    NVIC -> ISER[0] = (1 << TIM15_IRQn);
}

//===========================================================================
// Initialize the SPI2 peripheral.
//===========================================================================
void init_spi2(void)
{
    //enable ports 12,13,15 for alternate function
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB -> MODER &= ~0xcf000000;
    GPIOB -> MODER |= 0x8a000000;

    GPIOB -> AFR[1] &= ~0xf0ff0000; //set pb15, 13, 12 to AF0
    GPIOB -> AFR[1] |= -0x00000000;


    RCC->APB1ENR |= (1 << 14); //enable spi2

    SPI2 -> CR1 &= ~(1 << 6);
    SPI2 -> CR1 &= ~0x38;
    SPI2 -> CR1 |= 0x38; //set the baud rate as low
    SPI2 -> CR1 |= (1 << 2); // Master Mode
    SPI2 -> CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_0 | SPI_CR2_SSOE | SPI_CR2_NSSP;
    SPI2 -> CR1 |= (1 << 6); //enable periph


}

//===========================================================================
// Configure the SPI2 peripheral to trigger the DMA channel when the
// transmitter is empty.
//===========================================================================
void spi2_setup_dma(void) {
    setup_dma();
    SPI2->CR2 |= SPI_CR2_TXDMAEN;// Transfer register empty DMA enable
    SPI2->CR2 |= (1 << 7); //TXIEIE unmask interrupt

}

//===========================================================================
// Enable the DMA channel.
//===========================================================================
void spi2_enable_dma(void) {
    enable_dma();
}

//===========================================================================
// 2.4 SPI OLED Display
//===========================================================================
void setup_spi1() {
    // PA5  SPI1_SCK
    // PA6  SPI1_MISO
    // PA7  SPI1_MOSI

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;



    GPIOA -> AFR[0] &= ~0xf0f00000; //set pa5 and pa7 to AF)
    GPIOA -> AFR[0] |= 0x00000000;

    GPIOB -> MODER &= ~0xf00000;
    GPIOB -> MODER |=  0x500000;//pb 10 pb 11 output

    GPIOA -> MODER |= 0x10;

    GPIOB -> ODR |= (1 << 10);
    GPIOB -> ODR |= (1 << 11);
    GPIOA -> ODR |= (1 << 3);




    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    SPI1 -> CR1 &= ~(1 << 6);
    //baud rate as high as possible
    //SPI1 -> CR1 |= 0x38;
    //PI1->CR1 &= ~(SPI_CR1_BR);


    SPI1->CR1 &= ~(SPI_CR1_BR); //baud rate as high as possible
	SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
	SPI1->CR2 = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
	SPI1->CR1 |= SPI_CR1_SPE;




}

 /* void setup_spi1()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5
            | GPIO_MODER_MODER7 | GPIO_MODER_MODER2  | GPIO_MODER_MODER3);
    GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1
            | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0;

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~(SPI_CR1_BR);
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;
    SPI1->CR2 = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0 | SPI_CR2_SSOE | SPI_CR2_NSSP;
    SPI1->CR1 |= SPI_CR1_SPE;

} */



void spi_cmd(unsigned int data) {
    while(!(SPI1->SR & SPI_SR_TXE)) {}
    SPI1->DR = data;
}

void spi2_cmd(unsigned int data) {
    while(!(SPI2->SR & SPI_SR_TXE)) {}
    SPI2->DR = data;
}
void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);
}
void spi2_data(unsigned int data) {
    spi2_cmd(data | 0x200);
}

void spi2_init_oled() {
    nano_wait(1000000);
    spi2_cmd(0x38);
    spi2_cmd(0x08);
    spi2_cmd(0x01);
    nano_wait(2000000);
    spi2_cmd(0x06);
    spi2_cmd(0x02);
    spi2_cmd(0x0c);
}
void spi1_init_oled() {
    nano_wait(1000000);
    spi_cmd(0x38);
    spi_cmd(0x08);
    spi_cmd(0x01);
    nano_wait(2000000);
    spi_cmd(0x06);
    spi_cmd(0x02);
    spi_cmd(0x0c);
}
void spi1_display1(const char *string) {
    spi_cmd(0x02);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}

void spi2_display1(const char *string) {
    spi2_cmd(0x02);
    while(*string != '\0') {
        spi2_data(*string);
        string++;
    }
}
void spi1_display2(const char *string) {
    spi_cmd(0xc0);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}

void spi2_display2(const char *string) {
    spi2_cmd(0xc0);
    while(*string != '\0') {
        spi2_data(*string);
        string++;
    }
}

void init_tim6(void) {
    TIM6->CR1 &= ~TIM_CR1_CEN;

    time_remaining = 0;
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 48000-1;
    TIM6->ARR = 1000-1;
    TIM6->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] = 1<<TIM6_DAC_IRQn;
    TIM6->CR1 |= TIM_CR1_CEN;
}

void TIM6_DAC_IRQHandler(void) {
	char stringy[20];
    TIM6->SR &= ~TIM_SR_UIF;
    time_remaining+=1;
    sprintf(stringy, "Time: %ds", time_remaining);
    //sprintf(score_string, "Level: %d", level_score);
    LCD_DrawString(140, 275, BLACK, WHITE,  (stringy), 16, 0);




    spi2_display2(stringy);




}







//===========================================================================
// This is the 34-entry buffer to be copied into SPI1.
// Each element is a 16-bit value that is either character data or a command.
// Element 0 is the command to set the cursor to the first position of line 1.
// The next 16 elements are 16 characters.
// Element 17 is the command to set the cursor to the first position of line 2.
//===========================================================================
uint16_t display[34] = {
        0x002, // Command to set the cursor at the first position line 1
        0x200+'E', 0x200+'C', 0x200+'E', 0x200+'3', 0x200+'6', + 0x200+'2', 0x200+' ', 0x200+'i',
        0x200+'s', 0x200+' ', 0x200+'t', 0x200+'h', + 0x200+'e', 0x200+' ', 0x200+' ', 0x200+' ',
        0x0c0, // Command to set the cursor at the first position line 2
        0x200+'c', 0x200+'l', 0x200+'a', 0x200+'s', 0x200+'s', + 0x200+' ', 0x200+'f', 0x200+'o',
        0x200+'r', 0x200+' ', 0x200+'y', 0x200+'o', + 0x200+'u', 0x200+'!', 0x200+' ', 0x200+' ',
};

//===========================================================================
// Configure the proper DMA channel to be triggered by SPI1_TX.
// Set the SPI1 peripheral to trigger a DMA when the transmitter is empty.
//===========================================================================
void spi1_setup_dma(void)
{
    RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CPAR = (uint32_t) 0x40013008;
    DMA1_Channel3->CMAR = (uint32_t) display;
    DMA1_Channel3->CCR |= DMA_CCR_CIRC;
    DMA1_Channel3->CCR |= DMA_CCR_DIR;
    DMA1_Channel3->CNDTR = 34;


    DMA1_Channel3->CCR |= 0x500;


    /*
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CPAR = (uint32_t) 0x4000380C;
    DMA1_Channel5->CMAR = (uint32_t) msg;
    DMA1_Channel5->CNDTR = 0x8;

    DMA1_Channel5->CCR |= DMA_CCR_DIR;
    DMA1_Channel5->CCR |= DMA_CCR_MINC;
    DMA1_Channel5->CCR |= 0x400;
    DMA1_Channel5->CCR |= 0x100;
    DMA1_Channel5->CCR |= 1<<5;*/

    SPI1->CR2 |= SPI_CR2_TXDMAEN;// Transfer register empty DMA enable
    SPI1->CR2 |= (1 << 7); //enable

}



//===========================================================================
// Enable the DMA channel triggered by SPI1_TX.
//===========================================================================
void spi1_enable_dma(void)
{
    DMA1_Channel3->CCR |= DMA_CCR_EN;
}

//===========================================================================
// Main function
//===========================================================================

int main(void)
{
    msg[0] |= font['E'];
    msg[1] |= font['C'];
    msg[2] |= font['E'];
    msg[3] |= font[' '];
    msg[4] |= font['3'];
    msg[5] |= font['6'];
    msg[6] |= font['2'];
    msg[7] |= font[' '];

    // This time, autotest always runs as an invisible aid to you.
    //autotest();

    // GPIO enable
    //enable_ports();
    SystemClock_Config();
    LCD_Setup();
    //LCD_DrawString(5, 300, BLACK, WHITE, "Time: 9 ", 16, 0);
    init_tim6();
    // setup keyboard
    //init_tim7();


    /*setup_spi1();
	LCD_Init();
	LCD_Clear(BLACK);
	LCD_DrawLine(10,20,100,200, WHITE);
	LCD_DrawRectangle(10,20,100,200, GREEN);
	LCD_DrawFillRectangle(120,20,220,200, RED);
	LCD_Circle(50, 260, 50, 1, BLUE);
	LCD_DrawFillTriangle(130,130, 130,200, 190,160, YELLOW);
	LCD_DrawChar(150,155, BLACK, WHITE, 'X', 16, 1);
	LCD_DrawString(140,60,  WHITE, BLACK, "ECE 362", 16, 0);
	LCD_DrawString(140,80,  WHITE, BLACK, "has the", 16, 1);
	LCD_DrawString(130,100, BLACK, GREEN, "best toys", 16, 0); */
	//LCD_DrawPicture(110,220,(const Picture *)&image)

    // LED array Bit Bang
//#define BIT_BANG
#if defined(BIT_BANG)
    setup_bb();
    drive_bb();
#endif

    // Direct SPI peripheral to drive LED display
//#define SPI_LEDS
#if defined(SPI_LEDS)
    init_spi2();
    setup_dma();
    enable_dma();
    init_tim15();
    show_keys();
#endif

    // LED array SPI
//#define SPI_LEDS_DMA
#if defined(SPI_LEDS_DMA)
    init_spi2();
    spi2_setup_dma();
    spi2_enable_dma();
    show_keys();
#endif

    // SPI OLED direct drive
#define SPI_OLED
#if defined(SPI_OLED)

    init_spi2();
    spi2_init_oled();
    spi2_display1("Metaporter");
    spi2_display2("Time 0s");

#endif

    // SPI
//#define SPI_OLED_DMA
#if defined(SPI_OLED_DMA)
    init_spi1();
    spi1_init_oled();
    spi1_setup_dma();
    spi1_enable_dma();
#endif

    // Game on!  The goal is to score 100 points.
    //game();
}
