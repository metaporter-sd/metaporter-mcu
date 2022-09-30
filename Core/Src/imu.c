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
#include "utilities.h"
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
            if (count > 1000000)//1000000
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
        //nano_wait(1000000000);	// caused weird errors
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


void imu_init(IMU* imu, uint8_t addr, uint8_t mode) {
	i2c1_init();
	imu->addr = addr;
	imu->x = imu->y = imu->z = imu->w = 0;

//	imu_set_op_mode(imu, IMU_MODE_CONFIG);
//	nano_wait(20000000);
//
//	imu_set_sys_trigger(imu, IMU_RST_SYS); // reset imu
//	nano_wait(30000000);
//
//	imu_set_power_mode(imu, IMU_POWER_MODE_NORMAL);
//	nano_wait(10000000);
//
//	imu_set_page(imu, 0x00);
//
//	imu_set_sys_trigger(imu, IMU_SELF_TST); // perform self test
//	nano_wait(400000000);

//	imu_set_sys_trigger(imu, IMU_RST_INT); // enable interrupt pin
//	imu_set_int_en(imu, IMU_ACC_BSX_DRDY); // enable interrupt
//	imu_set_int_msk(imu, IMU_ACC_BSX_DRDY); // triggers change on int pin

	imu_set_op_mode(imu, mode);
	nano_wait(10000000);
}

void imu_set_op_mode(IMU * imu, uint8_t mode) {
	uint8_t mode_data[] = {IMU_OPR_MODE_ADDR, mode};
//	uint8_t mode_data[2];
//	mode_data[0] = IMU_OPR_MODE_ADDR;
//	mode_data[1] = mode;

	i2c1_send_data(imu->addr, mode_data, sizeof(mode_data));

	//nano_wait(30000000);
}

void imu_set_int_en(IMU * imu, uint8_t val) {
	uint8_t data[] = {IMU_INT_EN, val};

	i2c1_send_data(imu->addr, data, sizeof(data));
}

void imu_set_int_msk(IMU * imu, uint8_t val) {
	uint8_t data[] = {IMU_INT_MSK, val};

	i2c1_send_data(imu->addr, data, sizeof(data));
}

void imu_set_power_mode(IMU * imu, uint8_t mode) {
	uint8_t mode_data[] = {IMU_PWR_MODE_ADDR, mode};

	i2c1_send_data(imu->addr, mode_data, sizeof(mode_data));

}

void imu_set_page(IMU * imu, uint8_t page) {
	uint8_t page_data[] = {IMU_PAGE_ID_ADDR, page};

	i2c1_send_data(imu->addr, page_data, sizeof(page_data));

}

void imu_set_sys_trigger(IMU * imu, uint8_t val) {
	uint8_t data[] = {IMU_SYS_TRIGGER_ADDR, val}; // set RST_SYS bit to reset system

	i2c1_send_data(imu->addr, data, sizeof(data));
}

void imu_get_quat(IMU * imu) {
	uint8_t reg_addr[] = {IMU_QUATERNION_DATA_W_LSB_ADDR}; // set register to read from

	i2c1_send_data(imu->addr, reg_addr, sizeof(reg_addr));

	uint8_t buffer[8];
	memset(buffer, 0, 8);

	/* Read quat data (8 bytes) */
	i2c1_recv_data(imu->addr, buffer, sizeof(buffer));
	imu->w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
	imu->x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
	imu->y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
	imu->z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

}

void imu_test(IMU * imu) {

	imu_get_quat(imu);
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
