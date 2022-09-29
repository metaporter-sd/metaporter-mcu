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

struct IMU {
	uint8_t ag_addr;
	uint8_t mag_addr;
	int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
	int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
	int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
	int16_t temperature; // Chip temperature
	float gBias[3], aBias[3], mBias[3];
	int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];

};

// default device addresses
#define IMU_ADDR 0x29 // default addr

void imu_init(IMU* imu);
void imu_init_accel(IMU* imu);
void imu_init_gyro(IMU* imu);
void imu_init_mag(IMU* imu);

void imu_read_accel(IMU* imu);
void imu_read_gyro(IMU* imu);
void imu_read_mag(IMU* imu);
void imu_read_temp(IMU* imu);

void imu_read_accel(IMU* imu);
void imu_read_gyro(IMU* imu);
void imu_read_mag(IMU* imu);
void imu_read_temp(IMU* imu);



void imu_set_accel_gyro_mode();
void imu_set_mag_mode();


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


#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_H_ */
