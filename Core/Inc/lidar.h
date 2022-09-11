/*
 * lidar.h
 *
 *  Created on: Sep 8, 2022
 *      Author: Jehan Shah
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lidar.h
  * @brief          : Header for lidar.c file.
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
#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f0xx_hal.h"
#include <stdint.h> // for uint8_t
#include <string.h> // for strlen() and strcmp()
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define LIDAR_ACQ_COMMAND_REG 0x00 // register to write to to get data
#define LIDAR_ACQ_COMMAND_VAL 0x04
#define LIDAR_STATUS_REG 0x01
#define LIDAR_DIST_ADDR 0x8F
#define LIDAR_TIMEOUT_VAL 9999
#define DATA_BFR_SIZE 1
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */
void lidar_init();
void lidar_get_distance(uint16_t* pdist, uint8_t len);
void nano_wait(unsigned int n);

void lidar_test_start_stop();
void lidar_test_send_one(); // test sending one byte
void lidar_test_send_many(); // test multi-byte sending data once
void lidar_test_read_one(); // one read from one register including write setups
void lidar_wait_for_data();
void lidar_test_get_one_distance(); // gets one distance reading

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
void I2C1_init(); // setup the I2C channel 1 subsystem
void I2C1_start(uint32_t devaddr, uint8_t size, uint8_t dir); // initiates transfer with slave device with r/w intent
void I2C1_stop(void); // sends the stop bit
void I2C1_wait_idle(void); // waits until bus is idle
int8_t I2C1_send_data(uint8_t devaddr, void *pdata, uint8_t size);
int8_t I2C1_recv_data(uint8_t devaddr, void *pdata, uint8_t size);

void lidar_init_dist_measure();
//void lidar_wait_for_data();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* INC_LIDAR_H_ */
