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
 * @file    fls110.c
 * @author  QST AE team
 * @version V0.1
 * @date    2021-04-13
 * @id      $Id$
 * @brief   This file provides the functions for FLS110 sensor evaluation.
 *          |----------------------|
 *          |--F401RE---|--FLS110--|
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
#include "bsp_uart.h"
#include "fls110.h"

/******************************************************
 *                      Macros
 ******************************************************/
//#define __weak __attribute__((weak))
#define I2C_SLAVE_ADDR                0x31
#define I2C_SLAVE_ADDR_READ           ((I2C_SLAVE_ADDR<<1)|1)
#define I2C_SLAVE_ADDR_WRITE          ((I2C_SLAVE_ADDR<<1)|0)
#define FLS110_TIMEOUT                0xFF //I2C operation timout in ms
#define FLS110_REG_FW_ID              0x10
#define FLS110_REG_UNIQUE_ID          0x11
#define FLS110_REG_FW_BUILD           0x12
#define FLS110_REG_FW_RELEASE         0x13
#define FLS110_REG_AVG_WINDOW         0x20
#define FLS110_REG_MODE               0x21
#define FLS110_REG_READY              0x2F
#define FLS110_REG_READING            0x40
#define FLS110_REG_SENSOR_H           0x41
#define FLS110_REG_SENSOR_PWR         0x42
#define FLS110_REG_FLOW_TEMP          0x43
#define FLS110_REG_P_FLOW             0xE0
#define FLS110_REG_RH_FLOW            0xE1
#define FLS110_REG_CAL_TEMP           0xFC
#define FLS110_REG_CAL0               0xF0
#define FLS110_REG_CAL1               0xF1
#define FLS110_REG_CAL2               0xF2
#define FLS110_REG_CAL3               0xF3
#define FLS110_REG_CALIBRATE          0xFF
#define FLS110_REG_C1                 0xC1
#define FLS110_REG_C2                 0xC2
#define FLS110_REG_C3                 0xC3
#define FLS110_REG_BASIS              0xCB
#define FLS110_FW_ID                  0x0F100A

typedef enum {
  FLS_MODE_IDLE,
  FLS_MODE_CONTINUOUS,
  FLS_MODE_SINGLESHOT
}fls_mode_t;

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
void FLS_I2C1_Init(void)
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

static HAL_StatusTypeDef FLS_I2C_Read(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;
  //Status = HAL_I2C_Mem_Read_2(&hi2c1, I2C_SLAVE_ADDR_READ, pData, size, FLS110_TIMEOUT);
  Status = HAL_I2C_Master_Receive(&hi2c1, I2C_SLAVE_ADDR_READ, pData, size, FLS110_TIMEOUT);
  return Status;
}

HAL_StatusTypeDef FLS_I2C_Write(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;

  Status = HAL_I2C_Master_Transmit(&hi2c1, I2C_SLAVE_ADDR_WRITE, pData, size, FLS110_TIMEOUT);
  return Status;
}

static HAL_StatusTypeDef FLS_Reg_Read(uint8_t reg, uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;

  Status = FLS_I2C_Write(&reg, 1);
  if (Status != HAL_OK) {
    return Status;
  }

  Status = FLS_I2C_Read(pData, size);
  return Status;
}

uint8_t FLS110_data_read(uint8_t *pdata)
{
  uint8_t	i2c_buf[20]={0};
  uint8_t error = 0;

  if(pdata == NULL)
  {
    return HAL_ERROR;
  }
  error = FLS_I2C_Read(i2c_buf, 4);
  //printf("FLS_I2C_Read(%d):0x%02X,0x%02X,0x%02X,0x%02X\n",\
  //        error, i2c_buf[0], i2c_buf[1], i2c_buf[2], i2c_buf[3]);
  memmove(pdata, i2c_buf, 4);

  return error;
}

void FLS110_Check_FW_ID(void)
{
  //uint32_t readoutFwId = 0;
  uint8_t regVal[4] = {0,};
  HAL_StatusTypeDef ret;

  ret = FLS_Reg_Read(FLS110_REG_FW_ID, regVal, sizeof(regVal));
  printf("FW_ID: %02X,%02X,%02X,%02X\n", regVal[0],regVal[1],regVal[2],regVal[3]);
  //readoutFwId = regVal[0]
}

void FLS110_Check_Unique_ID(void)
{
  uint8_t regVal[12] = {0,};
  HAL_StatusTypeDef ret;

  ret = FLS_Reg_Read(FLS110_REG_UNIQUE_ID, regVal, sizeof(regVal));
  printf("Processor ID: ");
  for(uint8_t i=0; i<sizeof(regVal); i++) {
    printf("%02X,", regVal[i]);
  }
  printf("\n");
}

void FLS110_Sensor_Test(void)
{
  FLS110_Check_FW_ID();
  //HAL_Delay(500);
  FLS110_Check_Unique_ID();
}
