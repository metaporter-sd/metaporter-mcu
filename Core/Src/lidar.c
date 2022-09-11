/*
 * lidar.c
 *
 *  Created on: Sep 8, 2022
 *      Author: Jehan Shah
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lidar.c
  * @brief          : functions to receive data from lidar via I2C
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
#include "lidar.h"
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
void I2C1_init()
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

void I2C1_start(uint32_t devaddr, uint8_t size, uint8_t dir)
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

void I2C1_stop(void)
{
    if (I2C1->ISR & I2C_ISR_STOPF)
    {
        return;
    }
    I2C1->CR2 |= I2C_CR2_STOP; // Send stop bit as master

    while( (I2C1->ISR & I2C_ISR_STOPF) == 0); // Wait while stop flag is not set
    I2C1->ICR |= I2C_ICR_STOPCF; // Clear stop flag
}

void I2C1_wait_idle(void)
{
    while ((I2C1->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY); // Wait while busy
}

int8_t I2C1_send_data(uint8_t devaddr, void *pdata, uint8_t size)
{
    int i;
    if (size <= 0 || pdata == 0)
    {
        return -1;
    }
    uint8_t *udata = (uint8_t*)pdata;
    I2C1_wait_idle();

    I2C1_start(devaddr, size, 0);

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
                I2C1_stop();
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

    I2C1_stop();
    return 0;
}

int8_t I2C1_recv_data(uint8_t devaddr, void *pdata, uint8_t size)
{
    int i;
    if (size <= 0 || pdata == 0)
    {
        return -1;
    }
    uint8_t *udata = (uint8_t*) pdata;
    I2C1_wait_idle();

    I2C1_start(devaddr, size, 1);

    for (i = 0; i < size; i++)
    {
        while ((I2C1->ISR & I2C_ISR_RXNE) == 0);
        udata[i] = I2C1->RXDR & I2C_RXDR_RXDATA;
        //nano_wait(1000000);
    }

    while ((I2C1->ISR & I2C_ISR_TC) == 0);
    I2C1_stop();
    return 0;
}

void lidar_init()
{
	I2C1_init();
}

void lidar_init_dist_measure()
{
	uint8_t init_data[] = {LIDAR_ACQ_COMMAND_REG, LIDAR_ACQ_COMMAND_VAL};
	I2C1_send_data(lidar_addr, init_data, sizeof(init_data));
}

void lidar_wait_for_data()
{

	uint8_t reg_addr[] = {LIDAR_STATUS_REG};
	I2C1_send_data(lidar_addr, reg_addr, sizeof(reg_addr));

	uint16_t counter = 0;
	uint8_t busy[] = {1};
	while (busy[0])
	{
		if (counter > LIDAR_TIMEOUT_VAL)
		{
			break;
		}

		I2C1_recv_data(lidar_addr, busy, sizeof(busy));
		busy[0] &= 0x01;
		counter++;
		nano_wait(10000);
	}
}

void lidar_get_distance(uint16_t* pdist, uint8_t len)
{
	int i;
	for (i = 0; i < len; i++)
	{
		// step 1: initiate the transaction and set the register from which we want to read
		lidar_init_dist_measure();

		// step 2: wait till status reg lsb goes low
		lidar_wait_for_data();

		uint8_t reg_addr[] = {LIDAR_DIST_ADDR};
		I2C1_send_data(lidar_addr, reg_addr, sizeof(reg_addr));

		uint8_t temp[2] = {0};
		I2C1_recv_data(lidar_addr, temp, sizeof(temp));

		pdist[i] = ((temp[0] << 8) | temp[1]);
	}
}

void lidar_test_start_stop()
{
	while(1)
	{
		I2C1_wait_idle();
		I2C1_start(lidar_addr, 0, 0);
		int x = 0;
		while ((I2C1->ISR & I2C_ISR_TC) == 0 &&
				(I2C1->ISR & I2C_ISR_STOPF) == 0 &&
				(I2C1->ISR & I2C_ISR_NACKF) == 0)
			x++;
		if (I2C1->ISR & I2C_ISR_NACKF)
			I2C1->ICR |= I2C_ICR_NACKCF;
		if (I2C1->ISR & I2C_ISR_STOPF)
			I2C1->ICR |= I2C_ICR_STOPCF;
		else
			I2C1_stop();
		nano_wait(1000000);
	}

}

void lidar_test_send_one() // test sending data once
{
	uint8_t init_data[1];
	init_data[0] = LIDAR_ACQ_COMMAND_REG;
	I2C1_send_data(lidar_addr, init_data, sizeof(init_data));
}

void lidar_test_send_many() // test sending data once
{
	lidar_init_dist_measure();
}

void lidar_test_read_one()
{
	//lidar_init_dist_measure();
	uint8_t reg_addr[] = {LIDAR_STATUS_REG};
	I2C1_send_data(lidar_addr, reg_addr, sizeof(reg_addr));

	uint8_t busy[] = {1};
	I2C1_recv_data(lidar_addr, busy, sizeof(busy));
}

void lidar_test_get_one_distance()
{
	uint16_t dist[1] = {0};
	lidar_get_distance(dist, sizeof(dist) / sizeof(dist[0]));
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


