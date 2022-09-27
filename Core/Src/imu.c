/*
 * imu.c
 *
 *  Created on: Sep 26, 2022
 *      Author: Jehan Shah
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : imu.c
  * @brief          : functions to receive data from imu via I2C
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
#include "imu.h"
#include "imu_registers.h"
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
uint8_t lidar_addr = 0x62; // default 7 bit address
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void i2c1_init()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOB->AFR[0] |= (1 << 4*6) | (1 << 4 * 7);

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    I2C1->CR1 &= ~(I2C_CR1_PE | I2C_CR1_ANFOFF | I2C_CR1_ERRIE | I2C_CR1_NOSTRETCH);

    I2C1->TIMINGR = 0;
    I2C1->TIMINGR &= ~I2C_TIMINGR_PRESC;
    I2C1->TIMINGR |= 3 << 20;
    I2C1->TIMINGR |= 1 << 16;
    I2C1->TIMINGR |= 3 << 8;
    I2C1->TIMINGR |= 9 << 0;

    I2C1->OAR1 &= ~I2C_OAR1_OA1EN;
    I2C1->OAR2 &= ~I2C_OAR2_OA2EN;

    I2C1->CR2 &= ~I2C_CR2_ADD10;
    I2C1->CR2 |= I2C_CR2_AUTOEND;

    I2C1->CR1 |= I2C_CR1_PE;
}

void i2c1_start(uint32_t devaddr, uint8_t size, uint8_t dir)
{
	uint32_t tempreg = I2C1->CR2;
	tempreg &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD |
	        I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);
	if (dir == 1)
    {
		tempreg |= I2C_CR2_RD_WRN;
	}
    else
    {
        tempreg &= ~I2C_CR2_RD_WRN;
    }
    tempreg |= ((devaddr << 1) & I2C_CR2_SADD) | ((size << 16) & I2C_CR2_NBYTES);
    tempreg |= I2C_CR2_START;
    I2C1->CR2 = tempreg;
}

void i2c1_stop(void)
{
    if (I2C1->ISR & I2C_ISR_STOPF)
    {
        return;
    }
    I2C1->CR2 |= I2C_CR2_STOP; // Send stop bit as master

    while( (I2C1->ISR & I2C_ISR_STOPF) == 0); // Wait while stop flag is not set
    I2C1->ICR |= I2C_ICR_STOPCF; // Clear stop flag
}

void i2c1_wait_idle(void)
{
    while ((I2C1->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY); // Wait while busy
}

int8_t i2c1_send_data(uint8_t devaddr, void *pdata, uint8_t size)
{
    int i;
    if (size <= 0 || pdata == 0)
    {
        return -1;
    }
    uint8_t *udata = (uint8_t*)pdata;
    i2c1_wait_idle();

    i2c1_start(devaddr, size, 0);

    for (i = 0; i < size; i++)
    {
        int count = 0;
        while ((I2C1->ISR & I2C_ISR_TXIS) == 0)
        {
            count += 1;
            if (count > 1000000)
            {
                return -1;
            }

            if ((I2C1->ISR & I2C_ISR_NACKF))
            {
                I2C1->ICR |= I2C_ICR_NACKCF;
                i2c1_stop();
                return -1;
            }
        }

        I2C1->TXDR = udata[i] & I2C_TXDR_TXDATA;
        //nano_wait(1000000);
    }

    while ((I2C1->ISR & I2C_ISR_TC) == 0 && (I2C1->ISR & I2C_ISR_NACKF) == 0);

    if ((I2C1->ISR & I2C_ISR_NACKF) != 0)
    {
        return -1;
    }

    i2c1_stop();
    return 0;
}

int8_t i2c1_recv_data(uint8_t devaddr, void *pdata, uint8_t size)
{
    int i;
    if (size <= 0 || pdata == 0)
    {
        return -1;
    }
    uint8_t *udata = (uint8_t*) pdata;
    i2c1_wait_idle();

    i2c1_start(devaddr, size, 1);

    for (i = 0; i < size; i++)
    {
        while ((I2C1->ISR & I2C_ISR_RXNE) == 0);
        udata[i] = I2C1->RXDR & I2C_RXDR_RXDATA;
        //nano_wait(1000000);
    }

    while ((I2C1->ISR & I2C_ISR_TC) == 0);
    i2c1_stop();
    return 0;
}

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}


/* USER CODE END 0 */


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
