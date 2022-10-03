
//============================================================================
// lcd.c: Adapted from the lcdwiki.com examples.
//============================================================================

#include "stm32f0xx.h"
#include <stdint.h>
#include "lcd.h"
#include "utilities.h"

lcd_dev_t lcddev;

#define SPI SPI1

#define CS_NUM  8
#define CS_BIT  (1<<CS_NUM)
#define CS_HIGH do { GPIOB->BSRR = GPIO_BSRR_BS_8; } while(0)
#define CS_LOW do { GPIOB->BSRR = GPIO_BSRR_BR_8; } while(0)
#define RESET_NUM 11
#define RESET_BIT (1<<RESET_NUM)
#define RESET_HIGH do { GPIOB->BSRR = GPIO_BSRR_BS_11; } while(0)
#define RESET_LOW  do { GPIOB->BSRR = GPIO_BSRR_BR_11; } while(0)
#define DC_NUM 14
#define DC_BIT (1<<DC_NUM)
#define DC_HIGH do { GPIOB->BSRR = GPIO_BSRR_BS_14; } while(0)
#define DC_LOW  do { GPIOB->BSRR = GPIO_BSRR_BR_14; } while(0)

// Set the CS pin low if val is non-zero.
// Note that when CS is being set high again, wait on SPI to not be busy.
static void tft_select(int val)
{
    if (val == 0) {
        while(SPI1->SR & SPI_SR_BSY)
            ;
        CS_HIGH;
    } else {
        while((GPIOB->ODR & (CS_BIT)) == 0) {
            ; // If CS is already low, this is an error.  Loop forever.
            // This has happened because something called a drawing subroutine
            // while one was already in process.  For instance, the main()
            // subroutine could call a long-running LCD_DrawABC function,
            // and an ISR interrupts it and calls another LCD_DrawXYZ function.
            // This is a common mistake made by students.
            // This is what catches the problem early.
        }
        CS_LOW;
    }
}

// If val is non-zero, set nRESET low to reset the display.
static void tft_reset(int val)
{
    if (val) {
        RESET_LOW;
    } else {
        RESET_HIGH;
    }
}

// If
static void tft_reg_select(int val)
{
    if (val == 1) { // select registers
        DC_LOW; // clear
    } else { // select data
        DC_HIGH; // set
    }
}

void lcd_reset(void)
{
    lcddev.reset(1);      // Assert reset
    nano_wait(100000000); // Wait
    lcddev.reset(0);      // De-assert reset
    nano_wait(50000000);  // Wait
}

// Write to an LCD "register"
void lcd_wr_reg(uint8_t data)
{
    while((SPI->SR & SPI_SR_BSY) != 0)
        ;
    // Don't clear RS until the previous operation is done.
    lcddev.reg_select(1);
    *((uint8_t*)&SPI->DR) = data;
}

// Write 8-bit data to the LCD
void lcd_wr_data(uint8_t data)
{
    while((SPI->SR & SPI_SR_BSY) != 0)
        ;
    // Don't set RS until the previous operation is done.
    lcddev.reg_select(0);
    *((uint8_t*)&SPI->DR) = data;
}

// Prepare to write 16-bit data to the LCD
void lcd_write_data_16_prepare()
{
    lcddev.reg_select(0);
    SPI->CR2 |= SPI_CR2_DS;
}

// Write 16-bit data
void lcd_write_data_16(u16 data)
{
    while((SPI->SR & SPI_SR_TXE) == 0)
        ;
    SPI->DR = data;
}

// Finish writing 16-bit data
void lcd_write_data_16_end()
{
    SPI->CR2 &= ~SPI_CR2_DS; // bad value forces it back to 8-bit mode
}
#endif /* not SLOW_SPI */

// Select an LCD "register" and write 8-bit data to it.
void lcd_write_reg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
    lcd_wr_reg(LCD_Reg);
    lcd_wr_data(LCD_RegValue);
}

// Issue the "write RAM" command configured for the display.
void lcd_write_ram_prepare(void)
{
    lcd_wr_reg(lcddev.wramcmd);
}

// Configure the lcddev fields for the display orientation.
void lcd_direction(u8 direction)
{
    lcddev.setxcmd=0x2A;
    lcddev.setycmd=0x2B;
    lcddev.wramcmd=0x2C;
    switch(direction){
    case 0:
        lcddev.width=LCD_W;
        lcddev.height=LCD_H;
        lcd_write_reg(0x36,(1<<3)|(0<<6)|(0<<7));//BGR==1,MY==0,MX==0,MV==0
        break;
    case 1:
        lcddev.width=LCD_H;
        lcddev.height=LCD_W;
        lcd_write_reg(0x36,(1<<3)|(0<<7)|(1<<6)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
        break;
    case 2:
        lcddev.width=LCD_W;
        lcddev.height=LCD_H;
        lcd_write_reg(0x36,(1<<3)|(1<<6)|(1<<7));//BGR==1,MY==0,MX==0,MV==0
        break;
    case 3:
        lcddev.width=LCD_H;
        lcddev.height=LCD_W;
        lcd_write_reg(0x36,(1<<3)|(1<<7)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
        break;
    default:break;
    }
}

// Do the initialization sequence for the display.
void lcd_init(void (*reset)(int), void (*select)(int), void (*reg_select)(int))
{
    lcddev.reset = tft_reset;
    lcddev.select = tft_select;
    lcddev.reg_select = tft_reg_select;
    if (reset)
        lcddev.reset = reset;
    if (select)
        lcddev.select = select;
    if (reg_select)
        lcddev.reg_select = reg_select;
    lcddev.select(1);
    lcd_reset();
    // Initialization sequence for 2.2inch ILI9341
    lcd_wr_reg(0xCF);
    lcd_wr_data(0x00);
    lcd_wr_data(0xD9); // C1
    lcd_wr_data(0X30);
    lcd_wr_reg(0xED);
    lcd_wr_data(0x64);
    lcd_wr_data(0x03);
    lcd_wr_data(0X12);
    lcd_wr_data(0X81);
    lcd_wr_reg(0xE8);
    lcd_wr_data(0x85);
    lcd_wr_data(0x10);
    lcd_wr_data(0x7A);
    lcd_wr_reg(0xCB);
    lcd_wr_data(0x39);
    lcd_wr_data(0x2C);
    lcd_wr_data(0x00);
    lcd_wr_data(0x34);
    lcd_wr_data(0x02);
    lcd_wr_reg(0xF7);
    lcd_wr_data(0x20);
    lcd_wr_reg(0xEA);
    lcd_wr_data(0x00);
    lcd_wr_data(0x00);
    lcd_wr_reg(0xC0);    // Power control
    lcd_wr_data(0x21);   // VRH[5:0]  //1B
    lcd_wr_reg(0xC1);    // Power control
    lcd_wr_data(0x12);   // SAP[2:0];BT[3:0] //01
    lcd_wr_reg(0xC5);    // VCM control
    lcd_wr_data(0x39);   // 3F
    lcd_wr_data(0x37);   // 3C
    lcd_wr_reg(0xC7);    // VCM control2
    lcd_wr_data(0XAB);   // B0
    lcd_wr_reg(0x36);    // Memory Access Control
    lcd_wr_data(0x48);
    lcd_wr_reg(0x3A);
    lcd_wr_data(0x55);
    lcd_wr_reg(0xB1);
    lcd_wr_data(0x00);
    lcd_wr_data(0x1B);   // 1A
    lcd_wr_reg(0xB6);    // Display Function Control
    lcd_wr_data(0x0A);
    lcd_wr_data(0xA2);
    lcd_wr_reg(0xF2);    // 3Gamma Function Disable
    lcd_wr_data(0x00);
    lcd_wr_reg(0x26);    // Gamma curve selected
    lcd_wr_data(0x01);

    lcd_wr_reg(0xE0);     // Set Gamma
    lcd_wr_data(0x0F);
    lcd_wr_data(0x23);
    lcd_wr_data(0x1F);
    lcd_wr_data(0x0B);
    lcd_wr_data(0x0E);
    lcd_wr_data(0x08);
    lcd_wr_data(0x4B);
    lcd_wr_data(0XA8);
    lcd_wr_data(0x3B);
    lcd_wr_data(0x0A);
    lcd_wr_data(0x14);
    lcd_wr_data(0x06);
    lcd_wr_data(0x10);
    lcd_wr_data(0x09);
    lcd_wr_data(0x00);
    lcd_wr_reg(0XE1);      // Set Gamma
    lcd_wr_data(0x00);
    lcd_wr_data(0x1C);
    lcd_wr_data(0x20);
    lcd_wr_data(0x04);
    lcd_wr_data(0x10);
    lcd_wr_data(0x08);
    lcd_wr_data(0x34);
    lcd_wr_data(0x47);
    lcd_wr_data(0x44);
    lcd_wr_data(0x05);
    lcd_wr_data(0x0B);
    lcd_wr_data(0x09);
    lcd_wr_data(0x2F);
    lcd_wr_data(0x36);
    lcd_wr_data(0x0F);
    lcd_wr_reg(0x2B);
    lcd_wr_data(0x00);
    lcd_wr_data(0x00);
    lcd_wr_data(0x01);
    lcd_wr_data(0x3f);
    lcd_wr_reg(0x2A);
    lcd_wr_data(0x00);
    lcd_wr_data(0x00);
    lcd_wr_data(0x00);
    lcd_wr_data(0xef);
    lcd_wr_reg(0x11);     // Exit Sleep
    nano_wait(120000000); // Wait 120 ms
    lcd_wr_reg(0x29);     // Display on

    lcd_direction(USE_HORIZONTAL);
    lcddev.select(0);
}

void init_spi1(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER11 | GPIO_MODER_MODER14);
    GPIOB->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER14_0;
    GPIOB->ODR |= GPIO_ODR_8 | GPIO_ODR_11 | GPIO_ODR_14;
    GPIOB->MODER |= GPIO_MODER_MODER3 | GPIO_MODER_MODER5;
    GPIOB->MODER &= ~(GPIO_MODER_MODER3_0 | GPIO_MODER_MODER5_0);

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 &= ~SPI_CR1_BR;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE;
}

void lcd_setup() {
    init_spi1();
    tft_select(0);
    tft_reset(0);
    tft_reg_select(0);
    lcd_init(tft_reset, tft_select, tft_reg_select);
}

//===========================================================================
// Select a subset of the display to work on, and issue the "Write RAM"
// command to prepare to send pixel data to it.
//===========================================================================
void lcd_set_window(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
    lcd_wr_reg(lcddev.setxcmd);
    lcd_wr_data(xStart>>8);
    lcd_wr_data(0x00FF&xStart);
    lcd_wr_data(xEnd>>8);
    lcd_wr_data(0x00FF&xEnd);

    lcd_wr_reg(lcddev.setycmd);
    lcd_wr_data(yStart>>8);
    lcd_wr_data(0x00FF&yStart);
    lcd_wr_data(yEnd>>8);
    lcd_wr_data(0x00FF&yEnd);

    lcd_write_ram_prepare();
}

//===========================================================================
// Set the entire display to one color
//===========================================================================
void lcd_clear(u16 Color)
{
    lcddev.select(1);
    unsigned int i,m;
    lcd_set_window(0,0,lcddev.width-1,lcddev.height-1);
    lcd_write_data_16_prepare();
    for(i=0;i<lcddev.height;i++)
    {
        for(m=0;m<lcddev.width;m++)
        {
            lcd_write_data_16(Color);
        }
    }
    lcd_write_data_16_end();
    lcddev.select(0);
}

//===========================================================================
// Draw a single dot of color c at (x,y)
//===========================================================================
static void _lcd_draw_point(u16 x, u16 y, u16 c)
{
    lcd_set_window(x,y,x,y);
    lcd_write_data_16_prepare();
    lcd_write_data_16(c);
    lcd_write_data_16_end();
}

void lcd_draw_point(u16 x, u16 y, u16 c)
{
    lcddev.select(1);
    _lcd_draw_point(x,y,c);
    lcddev.select(0);
}

//===========================================================================
// Draw a line of color c from (x1,y1) to (x2,y2).
//===========================================================================
static void _lcd_draw_line(u16 x1, u16 y1, u16 x2, u16 y2, u16 c)
{
    u16 t;
    int xerr=0,yerr=0,delta_x,delta_y,distance;
    int incx,incy,uRow,uCol;

    delta_x=x2-x1;
    delta_y=y2-y1;
    uRow=x1;
    uCol=y1;
    if(delta_x>0)incx=1;
    else if(delta_x==0)incx=0;
    else {incx=-1;delta_x=-delta_x;}
    if(delta_y>0)incy=1;
    else if(delta_y==0)incy=0;
    else{incy=-1;delta_y=-delta_y;}
    if( delta_x>delta_y)distance=delta_x;
    else distance=delta_y;
    for(t=0;t<=distance+1;t++ )
    {
        _lcd_draw_point(uRow,uCol,c);
        xerr+=delta_x ;
        yerr+=delta_y ;
        if(xerr>distance)
        {
            xerr-=distance;
            uRow+=incx;
        }
        if(yerr>distance)
        {
            yerr-=distance;
            uCol+=incy;
        }
    }
}

void lcd_draw_line(u16 x1, u16 y1, u16 x2, u16 y2, u16 c)
{
    lcddev.select(1);
    _lcd_draw_line(x1,y1,x2,y2,c);
    lcddev.select(0);
}

//===========================================================================
// Display a single character at position x,y on the screen.
// fc,bc are the foreground,background colors
// num is the ASCII character number
// size is the height of the character (either 12 or 16)
// When mode is set, the background will be transparent.
//===========================================================================
void _lcd_draw_char(u16 x,u16 y,u16 fc, u16 bc, char num, u8 size, u8 mode)
{
    u8 temp;
    u8 pos,t;
    num=num-' ';
    lcd_set_window(x,y,x+size/2-1,y+size-1);
    if (!mode) {
        lcd_write_data_16_prepare();
        for(pos=0;pos<size;pos++) {
            if (size==12)
                temp=asc2_1206[num][pos];
            else
                temp=asc2_1608[num][pos];
            for (t=0;t<size/2;t++) {
                if (temp&0x01)
                    lcd_write_data_16(fc);
                else
                    lcd_write_data_16(bc);
                temp>>=1;

            }
        }
        lcd_write_data_16_end();
    } else {
        for(pos=0;pos<size;pos++)
        {
            if (size==12)
                temp=asc2_1206[num][pos];
            else
                temp=asc2_1608[num][pos];
            for (t=0;t<size/2;t++)
            {
                if(temp&0x01)
                    _lcd_draw_point(x+t,y+pos,fc);
                temp>>=1;
            }
        }
    }
}

void lcd_draw_char(u16 x,u16 y,u16 fc, u16 bc, char num, u8 size, u8 mode)
{
    lcddev.select(1);
    _lcd_draw_char(x,y,fc,bc,num,size,mode);
    lcddev.select(0);
}

//===========================================================================
// Display a string of characters starting at location x,y.
// fc,bc are the foreground,background colors.
// p is the pointer to the string.
// size is the height of the character (either 12 or 16)
// When mode is set, the background will be transparent.
//===========================================================================
void lcd_draw_string(u16 x,u16 y, u16 fc, u16 bg, const char *p, u8 size, u8 mode)
{
    lcddev.select(1);
    while((*p<='~')&&(*p>=' '))
    {
        if(x>(lcddev.width-1)||y>(lcddev.height-1))
        return;
        _lcd_draw_char(x,y,fc,bg,*p,size,mode);
        x+=size/2;
        p++;
    }
    lcddev.select(0);
}

void lcd_update_status(char * status) {

	lcd_clear(0xffffff);
	lcd_draw_string(5, 100, BLACK, WHITE, "Status: ", 16, 0);
	lcd_draw_string(70, 100, BLACK, WHITE, (status), 16, 0);

}
