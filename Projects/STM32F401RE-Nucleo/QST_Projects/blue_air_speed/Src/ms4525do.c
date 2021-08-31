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
 * @file    ms4525do.c
 * @author  QST AE team
 * @version V0.3
 * @date    2021-08-31
 * @id      $Id$
 * @brief   This file provides the functions for TE MS4525DO sensor evaluation.
 *          |----------------------|
 *          |--F401RE---|--MS4525--|
 *          |----------------------|
 *          |---PB6-----|---SCL----|
 *          |---PB7-----|---SDA----|
 *          |---+5V-----|--Supply--|
 * @note
 *
 *****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "bsp_uart.h"
#include "ms4525do.h"

/******************************************************
 *                      Macros
 ******************************************************/
//#define __weak __attribute__((weak))
#define PARTNUMBER                    MS4525DO_DS_5AI_0001_D //Supply 5V, A type, range ±1 psi, 
#define I2C_SLAVE_ADDR                0x28//0x28,0x36,0x46
#define I2C_SLAVE_ADDR_READ           ((I2C_SLAVE_ADDR<<1)|1)
#define I2C_SLAVE_ADDR_WRITE          ((I2C_SLAVE_ADDR<<1)|0)
#define MS4525DO_TIMEOUT              0xFF //I2C operation timout in ms
//#define MS4525DO_PMAX                 1.0f//20*90%, Range 20psi
//#define MS4525DO_PMIN                 -1.0f //20*10%
#define MS4525DO_FULL_SCALE           (float)16383 //14bit

typedef enum {
  MS_NORMAL_GOOD,
  MS_RESERVED,
  MS_STALE_DATA,
  MS_FAULT
}ms_status_t;

extern void Error_Handler(void);

/******************************************************
 *                 Global Variables
 ******************************************************/
extern I2C_HandleTypeDef hi2c1;
//extern bool ms4525Log;

/******************************************************
 *                 Static Variables
 ******************************************************/

/* Private function prototypes -----------------------------------------------*/


static HAL_StatusTypeDef MS_I2C_Read(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;
  //Status = HAL_I2C_Mem_Read_2(&hi2c1, I2C_SLAVE_ADDR_READ, pData, size, MS4525DO_TIMEOUT);
  Status = HAL_I2C_Master_Receive(&hi2c1, I2C_SLAVE_ADDR_READ, pData, size, MS4525DO_TIMEOUT);
  return Status;
}

HAL_StatusTypeDef MS_I2C_Write(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;

  Status = HAL_I2C_Master_Transmit(&hi2c1, I2C_SLAVE_ADDR_WRITE, pData, size, MS4525DO_TIMEOUT);
  return Status;
}

uint8_t MS4525DO_data_read(uint8_t *pdata)
{
  uint8_t	i2c_buf[20]={0};
  uint8_t error = 0;

  if(pdata == NULL)
  {
    return HAL_ERROR;
  }
  error = MS_I2C_Read(i2c_buf, 4);
  //printf("MS_I2C_Read(%d):0x%02X,0x%02X,0x%02X,0x%02X\n",\
  //        error, i2c_buf[0], i2c_buf[1], i2c_buf[2], i2c_buf[3]);
  memmove(pdata, i2c_buf, 4);

  return error;
}

void MS4525DO_Sensor_Update(uint16_t *raw, float *pPress, float *pTemp, uint32_t *pTime)
{
  uint8_t ret = 0xff;
  ms_status_t status = MS_FAULT;
  uint8_t databuf[4] = {0x00};
  const float P_min = -1.0f;
  const float P_max = 1.0f;
  const float PSI_to_Pa = 6894.757f;
  int16_t dp_raw = 0, temp_raw = 0;
  float temperature = 0, dp_psi = 0, dp_pa = 0;
  uint32_t timestamp = 0;

  if ((raw == NULL) || (pPress == NULL) || (pTemp == NULL) || (pTime == NULL)) {
    return;
  }

  ret = MS4525DO_data_read(databuf);

  //status bits conditions indicated by the 2 MSBs (Bit[15:14] of I2C data packet
  status = (ms_status_t)(databuf[0] >> 6);
  if ((ret != HAL_OK) || (status != MS_NORMAL_GOOD))
  {
    printf("ERROR[TE]: I2C ret %d, MS status %d\r\n", ret, status);
    return;
  }

  timestamp = HAL_GetTick();

  //Bit[13:0] 14-bit bridge data
  dp_raw = (databuf[0] << 8) + databuf[1];
  dp_raw = 0x3FFF & dp_raw;

  //Bit[15:5] 11-bit corrected temperature data
  temp_raw = (databuf[2] << 8) + databuf[3];
  temp_raw = (0xFFE0 & temp_raw) >> 5;

  /*
    this equation is an inversion of the equation in the
    pressure transfer function figure on page 4 of the datasheet

    We negate the result so that positive differential pressures
    are generated when the bottom port is used as the static
    port on the pitot and top port is used as the dynamic port
    */
  dp_psi = -((dp_raw - 0.1f*MS4525DO_FULL_SCALE) * (P_max - P_min)
              /(0.8f*MS4525DO_FULL_SCALE) + P_min);
  dp_pa = dp_psi * PSI_to_Pa;

  temperature = (float)temp_raw*200/2047 - 50; //came from datasheet page4/14

  *raw = dp_raw;
  *pPress = dp_pa;
  *pTemp = temperature;
  *pTime = timestamp;
  //if (ms4525Log) {
  //  printf("%ld: [TE(%d)], %d, %.2f, %.1f℃\n", timestamp, status, dp_raw, dp_pa, temperature);
  //}
}
