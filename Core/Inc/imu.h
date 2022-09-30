/*
 * imu.h
 *
 *  Created on: Sep 26, 2022
 *      Author: Jehan Shah
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : imu.h
  * @brief          : Header for imu.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_IMU_H_
#define INC_IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include <stdint.h> // for uint8_t
#include <string.h> // for strlen() and strcmp()

// default device addresses
#define IMU_ADDR 0x28 // default addr

// operation modes
#define IMU_MODE_CONFIG 0x00 // imu configuration
#define IMU_MODE_NDOF 0x0C // fusion mode with mag calibration

// power mode
#define IMU_POWER_MODE_NORMAL 0x00

// imu registers
#define IMU_PAGE_ID_ADDR 0x07
#define IMU_INT_MSK 0x0F
#define IMU_INT_EN 0x10
#define IMU_QUATERNION_DATA_W_LSB_ADDR 0x20
#define IMU_OPR_MODE_ADDR 0x3D
#define IMU_PWR_MODE_ADDR 0x3E
#define IMU_SYS_TRIGGER_ADDR 0x3F

// sys_trigger bit shift values
#define IMU_SELF_TST 1
#define IMU_RST_SYS 1<<5
#define IMU_RST_INT 1<<6

// int bit shift values
#define IMU_ACC_BSX_DRDY 1



typedef struct IMU {
	uint8_t addr;
	int16_t x, y, z, w; // x, y, z, and w axis readings of the fusion with mag
	uint8_t curr_page;
	uint16_t fw_ver;
} IMU;

void imu_init(IMU * imu, uint8_t addr, uint8_t mode);

void imu_set_op_mode(IMU * imu, uint8_t mode);

void imu_set_int_en(IMU * imu, uint8_t val);

void imu_set_int_msk(IMU * imu, uint8_t val);

void imu_set_power_mode(IMU * imu, uint8_t mode);

void imu_set_page(IMU * imu, uint8_t page);

void imu_set_sys_trigger(IMU * imu, uint8_t val);

void imu_get_quat(IMU * imu);

void imu_get_fw_ver(IMU * imu);

void imu_test(IMU * imu);


// steps: 
// enter config mode to change sensor range and bandwidth or power mode
// defaults power mode seems pretty good
// 
// UNIT_SEL register to select units
// Use NDOF mode for data col which gives abs orientation 
// 3.8.2 interrupt settings


void i2c1_init(); // setup the I2C channel 1 subsystem
void i2c1_start(uint32_t devaddr, uint8_t size, uint8_t dir); // initiates transfer with slave device with r/w intent
void i2c1_stop(void); // sends the stop bit
void i2c1_wait_idle(void); // waits until bus is idle
int8_t i2c1_send_data(uint8_t devaddr, void *pdata, uint8_t size);
int8_t i2c1_recv_data(uint8_t devaddr, void *pdata, uint8_t size);

void init_exti_pb2(void);
void init_tim7(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_H_ */
