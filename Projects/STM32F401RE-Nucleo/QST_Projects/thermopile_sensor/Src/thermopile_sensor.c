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
 * @file    thermopile_sensor.c
 * @author  QST AE team
 * @version V0.1
 * @date    2021-04-13
 * @id      $Id$
 * @brief   This file provides the functions for thermopile sensor evaluation.
 *
 * @note
 *
 *****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "bsp_uart.h"
#include "thermopile_sensor.h"

/******************************************************
 *                      Macros
 ******************************************************/
//#define __weak __attribute__((weak))
#define THERMO_SLAVE_ADDR             0x00 //STP3022
#define THERMO_SLAVE_ADDR_READ        ((THERMO_SLAVE_ADDR<<1)|1)
#define THERMO_SLAVE_ADDR_WRITE       ((THERMO_SLAVE_ADDR<<1)|0)
//#define THERMO_READ_CMD               0x01
#define THERMO_DATE_ID_REG            0x00
#define THERMO_CHIP_ID_REG            0x01
#define THERMO_CALI_1_T_REG           0x20
#define THERMO_CALI_1_S_REG           0x22
#define THERMO_CALI_3_S_REG           0x30
#define THERMO_CALI_DATA_NUM          8
#define THERMO_RAW_DATA_REG           0xA2 //raw_data
#define THERMO_TEMPERATURE_REG        0xA6 //temperature
#define THERMO_TIMEOUT                0xFF


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
void MX_I2C1_Init(void)
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

static HAL_StatusTypeDef Thermo_I2C_Read(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;
  Status = HAL_I2C_Master_Receive(&hi2c1, THERMO_SLAVE_ADDR_READ, pData, size, THERMO_TIMEOUT);

  return Status;
}

HAL_StatusTypeDef Thermo_I2C_Write(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;
  Status = HAL_I2C_Master_Transmit(&hi2c1, THERMO_SLAVE_ADDR_WRITE, pData, size, THERMO_TIMEOUT);
  //printf("%s:[", __FUNCTION__);
  //for(uint8_t i=0; i<size; i++) {
  //  printf("0x%02X,", *pData);
  //  pData++;
  //}
  //printf("]\n");

  return Status;
}

uint8_t Thermo_read_calibration(uint32_t* pCaliData, uint8_t size)
{
  uint8_t cmd_buf[3]={0}, cali_high_reg[5]={0}, cali_low_reg[5]={0};
  uint8_t err = HAL_ERROR;
  uint16_t cali_high=0, cali_low=0;
  uint32_t calibration[THERMO_CALI_DATA_NUM] = {0};

  if((pCaliData == NULL) || (size < THERMO_CALI_DATA_NUM))
  {
    return HAL_ERROR;
  }

  //printf("Calibration data:");
  for (uint8_t i=0; i<THERMO_CALI_DATA_NUM; i++)
  {
    if (i<5) {
      cmd_buf[0] = THERMO_CALI_1_T_REG + 2*i;
    } else {
      cmd_buf[0] = THERMO_CALI_3_S_REG + 2*(i-5);
    }
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    memset(cali_high_reg, 0x00, sizeof(cali_high_reg));
    err = Thermo_I2C_Write(cmd_buf, sizeof(cmd_buf));
    HAL_Delay(200);
    err = Thermo_I2C_Read(cali_high_reg, 4);
    printf("Reg[0x%02X](%d): 0x%02X,0x%02X,0x%02X,0x%02X\n",
            cmd_buf[0],err,cali_high_reg[0],cali_high_reg[1],cali_high_reg[2],cali_high_reg[3]);
    cali_high = (uint16_t)cali_high_reg[1]<<8 | cali_high_reg[2];

    HAL_Delay(200);
    cmd_buf[0]++; //MTP_Address + 1
    memset(cali_low_reg, 0x00, sizeof(cali_low_reg));
    err = Thermo_I2C_Write(cmd_buf, sizeof(cmd_buf));
    HAL_Delay(200);
    err = Thermo_I2C_Read(cali_low_reg, 4);
    printf("Reg[0x%02X](%d): 0x%02X,0x%02X,0x%02X,0x%02X\n",
            cmd_buf[0],err,cali_low_reg[0],cali_low_reg[1],cali_low_reg[2],cali_low_reg[3]);
    cali_low = (uint16_t)cali_low_reg[1]<<8 | cali_low_reg[2];

    calibration[i] = ((uint32_t)cali_high<<3) | (cali_low>>14);
    //printf("\n[%d] %d, \n", i, calibration[i]);
    HAL_Delay(200);
  }
  //printf("\n");
  if (err == HAL_OK) {
    memmove(pCaliData, calibration, size);
  }

  return err;
}

uint8_t Thermo_read_data(int32_t *pdata)
{
  uint8_t cmd_buf[3]={0}, raw_reg[5]={0}, temp_reg[5]={0};
  uint8_t err = HAL_ERROR;
  int32_t raw=0, temp=0;

  if(pdata == NULL)
  {
    return HAL_ERROR;
  }

  cmd_buf[0] = THERMO_RAW_DATA_REG;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = 0x00;
  HAL_Delay(200);
  err = Thermo_I2C_Write(cmd_buf, sizeof(cmd_buf));
  //printf("Thermo_I2C_Write(RAW):%d\n", err);
  HAL_Delay(200);
  err = Thermo_I2C_Read(raw_reg, 4);
  raw = ((int32_t)raw_reg[1]<<16 | raw_reg[2]<<8 | raw_reg[3])>>6; //S_DATA= A2HEX[23:6]
  //printf("Thermo_I2C_Read_Raw(%d):0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
  //        err, raw_reg[0], raw_reg[1], raw_reg[2], raw_reg[3]);

  HAL_Delay(200);
  cmd_buf[0] = THERMO_TEMPERATURE_REG;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = 0x00;
  err = Thermo_I2C_Write(cmd_buf, sizeof(cmd_buf));
  //printf("Thermo_I2C_Write(TEMP):%d\n", err);
  HAL_Delay(200);
  err = Thermo_I2C_Read(temp_reg, 4);
  temp = ((int32_t)temp_reg[1]<<16 | temp_reg[2]<<8 | temp_reg[3])>>6; //S_DATA= A6HEX[23:6]
  //printf("Thermo_I2C_Read_Temp(%d):0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
  //        err, temp_reg[0], temp_reg[1], temp_reg[2], temp_reg[3]);
  printf("[raw, temp], %ld, %ld\n", raw, temp);
  HAL_Delay(200);

  return err;
}

uint8_t Thermo_Sensor_Check_Chipid(void)
{
  uint8_t ret = HAL_ERROR;
  uint8_t cmd_buf[3]={0};
  uint8_t date_reg[4]={0}, pid_reg[4]={0}, years, months;
  uint32_t pid;

  cmd_buf[0] = THERMO_DATE_ID_REG;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = 0x00;
  HAL_Delay(200);
  ret = Thermo_I2C_Write(cmd_buf, sizeof(cmd_buf));
  HAL_Delay(200);
  ret = Thermo_I2C_Read(date_reg, 4);
  printf("Reg[0x%02X](%d): 0x%02X,0x%02X,0x%02X,0x%02X\n",
          cmd_buf[0],ret,date_reg[0],date_reg[1],date_reg[2],date_reg[3]);

  HAL_Delay(200);
  cmd_buf[0] = THERMO_CHIP_ID_REG;
  ret = Thermo_I2C_Write(cmd_buf, sizeof(cmd_buf));
  HAL_Delay(200);
  ret = Thermo_I2C_Read(pid_reg, 4);
  printf("Reg[0x%02X](%d): 0x%02X,0x%02X,0x%02X,0x%02X\n",
          cmd_buf[0],ret,pid_reg[0],pid_reg[1],pid_reg[2],pid_reg[3]);
  pid = (((uint32_t)date_reg[1]&0x07)<<16) | (pid_reg[2]<<8) | pid_reg[1];
  months = (((uint16_t)date_reg[2]<<8 | date_reg[1])>>3) & 0x3F; //Months=Bit[8:3]
  years = date_reg[2] >> 1; //Years=Bit[15:9]

  printf("Years: %d, Months: %d, ChipID:0x%06X\n", years, months, pid);
  HAL_Delay(200);

  return ret;
}

void Thermo_Sensor_Init(void)
{
  uint32_t calibration[THERMO_CALI_DATA_NUM] = {0};
  MX_I2C1_Init();
  HAL_Delay(200);
  Thermo_Sensor_Check_Chipid();
  Thermo_read_calibration(calibration, sizeof(calibration));
  printf("calibration:");
  for(uint8_t i=0; i<THERMO_CALI_DATA_NUM; i++)
  {
    printf("%d,", calibration[i]);
  }
  printf("\n");
}

void Thermo_Sensor_Test(void)
{
  uint8_t ret=HAL_ERROR;
  int32_t result = 0;
  ret = Thermo_read_data(&result);
  UNUSED(ret);
  //printf("raw(status %d): %d\n\n", ret, result);
}
