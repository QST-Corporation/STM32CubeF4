/**************************************************************************************************
 
  Shanghai QST Corporation confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Shanghai QST 
  Corporation ("QST"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of QST. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a QST Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  QST OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/

/******************************************************************************
 * @file    gxhtc3.c
 * @author  QST AE team
 * @version V0.1
 * @date    2021-04-13
 * @id      $Id$
 * @brief   This file provides the functions for gxhtc3 sensor evaluation.
 *          |----------------------|
 *          |--F401RE---|--gxhtc3--|
 *          |----------------------|
 *          |---PB6-----|---SCL----|
 *          |---PB7-----|---SDA----|
 *          |---+3.3V---|---VDD----|
 * @note
 *
 *****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "bsp_uart.h"
#include "gxhtc3.h"

/******************************************************
 *                      Macros
 ******************************************************/
//#define __weak __attribute__((weak))
#define I2C_SLAVE_ADDR                0x70
#define I2C_SLAVE_ADDR_READ           ((I2C_SLAVE_ADDR<<1)|1)
#define I2C_SLAVE_ADDR_WRITE          ((I2C_SLAVE_ADDR<<1)|0)
#define GXHTC3_TIMEOUT                0xFF //I2C operation timout in ms
#define GXHTC3_WAKEUP_CMD             0x3517
#define GXHTC3_SLEEP_CMD              0xB098
#define GXHTC3_NORMAL_READ_T_CMD      0x7CA2  //clock stretching disabled: 0x7866
#define GXHTC3_NORMAL_READ_RH_CMD     0x5C24  //clock stretching disabled: 0x58E0
#define GXHTC3_SW_RESET_CMD           0x805D
#define GXHTC3_READ_ID_CMD            0xEFC8
#define GXHTC3_ID                     0xFFFF //TBD

typedef enum {
  GXHTC_MODE_WAKEUP    = GXHTC3_WAKEUP_CMD,
  GXHTC_MODE_SLEEP     = GXHTC3_SLEEP_CMD
}gxhtc_mode_t;

typedef enum {
  GXHTC_MEAS_NORMAL_T_FIRST    = GXHTC3_NORMAL_READ_T_CMD,
  GXHTC_MEAS_NORMAL_RH_FIRST   = GXHTC3_NORMAL_READ_RH_CMD
}gxhtc_meas_mode_t;

typedef enum {
  GXHTC_SUCCESS    = 0x00U,
  GXHTC_ERROR      = 0x01U
}gxhtc_status_t;

extern void Error_Handler(void);

/******************************************************
 *                 Global Variables
 ******************************************************/
I2C_HandleTypeDef hi2c1;

/******************************************************
 *                 Static Variables
 ******************************************************/

/* Private function prototypes -----------------------------------------------*/
/* I2C1 init function */
void Gxhtc3_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;//GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);
  }
}

static HAL_StatusTypeDef Gxhtc3_I2C_Read(uint8_t *pData, uint16_t size)
{
  //return HAL_I2C_Mem_Read_2(&hi2c1, I2C_SLAVE_ADDR_READ, pData, size, GXHTC3_TIMEOUT);
  return HAL_I2C_Master_Receive(&hi2c1, I2C_SLAVE_ADDR_READ, pData, size, GXHTC3_TIMEOUT);
}

static HAL_StatusTypeDef Gxhtc3_I2C_Write(uint8_t *pData, uint16_t size)
{
  return HAL_I2C_Master_Transmit(&hi2c1, I2C_SLAVE_ADDR_WRITE, pData, size, GXHTC3_TIMEOUT);
}

static gxhtc_status_t Gxhtc3_Reg_Read(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef ret;
  ret = Gxhtc3_I2C_Read(pData, size);
  return ret == HAL_OK ? GXHTC_SUCCESS : GXHTC_ERROR;
}

static gxhtc_status_t Gxhtc3_Cmd_Transmit(uint16_t cmd)
{
  HAL_StatusTypeDef ret;
  uint8_t payload[2] = {0,};

  payload[0] = cmd>>8;
  payload[1] = cmd&0xFF;
  ret = Gxhtc3_I2C_Write(payload, 2);
  return ret == HAL_OK ? GXHTC_SUCCESS : GXHTC_ERROR;
}

static gxhtc_status_t Gxhtc3_Set_Mode(gxhtc_mode_t mode)
{
  HAL_StatusTypeDef ret;
  ret = Gxhtc3_Cmd_Transmit((uint16_t)mode);

  return ret == HAL_OK? GXHTC_SUCCESS : GXHTC_ERROR;
}

static gxhtc_status_t Gxhtc3_Start_Measurement(gxhtc_meas_mode_t mode)
{
  HAL_StatusTypeDef ret;
  ret = Gxhtc3_Cmd_Transmit((uint16_t)mode);

  return ret == HAL_OK? GXHTC_SUCCESS : GXHTC_ERROR;
}

static gxhtc_status_t Gxhtc3_Check_Device_Presence(void)
{
  uint8_t regVal[3] = {0,};
  uint16_t gxhtc3_id = 0;

  Gxhtc3_Cmd_Transmit(GXHTC3_READ_ID_CMD);
  HAL_Delay(1);
  Gxhtc3_Reg_Read(regVal, 3);
  gxhtc3_id = ((uint16_t)regVal[2]<<8)|regVal[1];
  printf("Readout ID 0x%02X\n", gxhtc3_id);

  if(gxhtc3_id == GXHTC3_ID) {
    printf("Sensor GXHTC3 exist\n");
    return GXHTC_SUCCESS;
  } else {
    printf("Sensor GXHTC3 absence! Readout ID %d\n", gxhtc3_id);
    return GXHTC_ERROR;
  }
}

void Gxhtc3_Sensor_Init(void)
{
  Gxhtc3_Cmd_Transmit(GXHTC3_SW_RESET_CMD);
  HAL_Delay(1);

  Gxhtc3_Check_Device_Presence();
  HAL_Delay(1);
  Gxhtc3_Set_Mode(GXHTC_MODE_SLEEP);
}

void Gxhtc3_Sensor_Test(void)
{
  gxhtc_status_t ret;
  uint8_t regVal[6] = {0};
  uint16_t rhRaw = 0, tempRaw = 0;
  float humidity = 0, temperture = 0;
  uint8_t rh_crc=0, temp_crc=0;

  //1, wakeup
  Gxhtc3_Set_Mode(GXHTC_MODE_WAKEUP);
  HAL_Delay(1);
  //2, measurement
  Gxhtc3_Start_Measurement(GXHTC_MEAS_NORMAL_RH_FIRST);
  HAL_Delay(1);

  //3, readout data
  ret = Gxhtc3_Reg_Read(regVal, 6);
  if (ret == GXHTC_SUCCESS) {
    rhRaw = ((uint16_t)regVal[0]<<8)|regVal[1];
    rh_crc = regVal[2];
    tempRaw = ((uint16_t)regVal[3]<<8)|regVal[4];
    temp_crc = regVal[5];

    humidity = (100.0f*rhRaw)/pow(2,16); //refer to spec p9
    temperture = (175.0f*tempRaw)/pow(2,16) - 45; //refer to spec p9
  }

  //4, sleep
  Gxhtc3_Set_Mode(GXHTC_MODE_SLEEP);

  printf("humidity %d, %f, temp %d, %.1f\n", rhRaw, humidity, tempRaw, temperture);
}
