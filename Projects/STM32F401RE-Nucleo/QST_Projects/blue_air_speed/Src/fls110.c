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
#define FLS110_MAX_PAYLOAD            0x0C //12U
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
  FLS_MODE_SINGLESHOT,
  FLS_MODE_ILLEGAL = 0xFFU
}fls_mode_t;

typedef enum {
  FLS_BASIS_MASSFLOW,
  FLS_BASIS_DP,
  FLS_BASIS_ILLEGAL = 0xFFU
}fls_basis_t;

typedef enum {
  FLS_SUCCESS    = 0x00U,
  FLS_ERROR      = 0x01U
}fls_status_t;

typedef enum {
  FLS_NOTREADY   = 0x00U,
  FLS_READY      = 0x01U,
  FLS_READY_ILLEGAL = 0xFFU
}fls_ready_t;

extern void Error_Handler(void);

/******************************************************
 *                 Global Variables
 ******************************************************/
extern I2C_HandleTypeDef hi2c1;

/******************************************************
 *                 Static Variables
 ******************************************************/

/* Private function prototypes -----------------------------------------------*/


static HAL_StatusTypeDef FLS_I2C_Read(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;
  //Status = HAL_I2C_Mem_Read_2(&hi2c1, I2C_SLAVE_ADDR_READ, pData, size, FLS110_TIMEOUT);
  Status = HAL_I2C_Master_Receive(&hi2c1, I2C_SLAVE_ADDR_READ, pData, size, FLS110_TIMEOUT);
  return Status;
}

static HAL_StatusTypeDef FLS_I2C_Write(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;

  Status = HAL_I2C_Master_Transmit(&hi2c1, I2C_SLAVE_ADDR_WRITE, pData, size, FLS110_TIMEOUT);
  return Status;
}

static HAL_StatusTypeDef FLS110_Reg_Read(uint8_t reg, uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;

  Status = FLS_I2C_Write(&reg, 1);
  if (Status != HAL_OK) {
    return Status;
  }

  return FLS_I2C_Read(pData, size);
}

static HAL_StatusTypeDef FLS110_Reg_Write(uint8_t reg, uint8_t *pData, uint16_t size)
{
  uint8_t payload[FLS110_MAX_PAYLOAD] = {0,};

  payload[0] = reg;
  memmove(&payload[1], pData, size);
  return FLS_I2C_Write(payload, size+1);
}

static fls_status_t FLS110_pflow_read(float *pdata)
{
  HAL_StatusTypeDef ret;
  uint8_t	regVal[4]={0};
  uint32_t regPflow = 0;
  float pflow = 0.0f;

  if(pdata == NULL)
  {
    return FLS_ERROR;
  }
  ret = FLS110_Reg_Read(FLS110_REG_P_FLOW, regVal, 4);
  memmove((uint8_t *)&regPflow, regVal, sizeof(regVal));
  pflow = (float)regPflow/256;

  printf("FLS110 Pflow(%d), %d, %.4fPa\n", ret, regPflow, pflow);
  *pdata = pflow;

  return ret == HAL_OK ? FLS_SUCCESS : FLS_ERROR;
}

static fls_status_t FLS110_data_read(float *pdata)
{
  HAL_StatusTypeDef ret;
  uint8_t	regVal[4]={0}, regTemp[2]={0};
  uint32_t reading = 0;
  float flowTemp = 0.0f;
  float pressure = 0.0f, psquare = 0.0f;
  float air_speed = 0.0f;

  if(pdata == NULL)
  {
    return FLS_ERROR;
  }
  ret = FLS110_Reg_Read(FLS110_REG_READING, regVal, 4);
  memmove((uint8_t *)&reading, regVal, sizeof(regVal));
  pressure = (float)reading/256;
  psquare = pressure*pressure;
  FLS110_Reg_Read(FLS110_REG_FLOW_TEMP, regTemp, sizeof(regTemp));
  flowTemp = (float)((int16_t)regTemp[1] << 8 | regTemp[0])/256;

  if(pressure < 40) {
    air_speed = 0;
  }
  else if(pressure < 80) {
    air_speed = (-0.0016f)*psquare + 0.2715f*pressure - 8.7785f;
  } else if(pressure < 450) {
    air_speed = (-0.00001f)*psquare + 0.0184f*pressure + 1.4547f;
  } else {
    air_speed = (-0.0000006f)*psquare + 0.0093f*pressure + 3.7301f;
  }
  printf("FLS110(%d), %d, %.4f, %.1f, %.1f℃\n", ret, reading, pressure, air_speed, flowTemp);
  *pdata = air_speed;

  return ret == HAL_OK ? FLS_SUCCESS : FLS_ERROR;
}

static fls_status_t FLS110_Check_FW_ID(void)
{
  //uint32_t readoutFwId = 0;
  uint8_t regVal[4] = {0,};
  HAL_StatusTypeDef ret;

  ret = FLS110_Reg_Read(FLS110_REG_FW_ID, regVal, sizeof(regVal));
  printf("FW_ID: %02X,%02X,%02X,%02X\n", regVal[0],regVal[1],regVal[2],regVal[3]);
  //readoutFwId = regVal[0]
  return ret == HAL_OK ? FLS_SUCCESS : FLS_ERROR;
}

static fls_status_t FLS110_Check_Unique_ID(void)
{
  uint8_t regVal[12] = {0,};
  HAL_StatusTypeDef ret;

  ret = FLS110_Reg_Read(FLS110_REG_UNIQUE_ID, regVal, sizeof(regVal));
  printf("Processor ID: ");
  for(uint8_t i=0; i<sizeof(regVal); i++) {
    printf("%02X,", regVal[i]);
  }
  printf("\n");
  return ret == HAL_OK ? FLS_SUCCESS : FLS_ERROR;
}

static fls_status_t FLS110_Set_Mode(fls_mode_t mode)
{
  HAL_StatusTypeDef ret;
  fls_mode_t readout_mode = FLS_MODE_ILLEGAL;
  ret = FLS110_Reg_Write(FLS110_REG_MODE, &mode, 1);
  //printf("Set FLS110 mode %d status:%d\n", mode, ret);
  if (ret != HAL_OK){
    printf("Set FLS110 mode %d failed\n", mode);
    return FLS_ERROR;
  }

  HAL_Delay(10);
  ret = FLS110_Reg_Read(FLS110_REG_MODE, &readout_mode, 1);
  //printf("Readout FLS110 mode %d status: %d\n", readout_mode, ret);
  if((ret != HAL_OK) || (readout_mode != mode))
  {
    printf("Verify FLS110 mode %d failed, readout: %d\n", mode, readout_mode);
    return FLS_ERROR;
  }
  printf("Set FLS110 mode %d success\n", mode);
  return FLS_SUCCESS;
}

static fls_status_t FLS110_Set_Avg(uint8_t avg)
{
  HAL_StatusTypeDef ret;
  uint8_t readout_avg = 0xFF;
  ret = FLS110_Reg_Write(FLS110_REG_AVG_WINDOW, &avg, 1);
  //printf("Set FLS110 Avg %d status:%d\n", avg, ret);
  if (ret != HAL_OK){
    printf("Set FLS110 Avg %d failed\n", avg);
    return FLS_ERROR;
  }

  HAL_Delay(10);
  ret = FLS110_Reg_Read(FLS110_REG_AVG_WINDOW, &readout_avg, 1);
  //printf("Readout FLS110 Avg %d status: %d\n", readout_avg, ret);
  if((ret != HAL_OK) || (readout_avg != avg))
  {
    printf("Verify FLS110 Avg %d failed, readout: %d\n", avg, readout_avg);
    return FLS_ERROR;
  }
  printf("Set FLS110 Avg %d success\n", avg);
  return FLS_SUCCESS;
}

static fls_status_t FLS110_Set_Basis(fls_basis_t basis)
{
  HAL_StatusTypeDef ret;
  fls_basis_t readout_basis = FLS_BASIS_ILLEGAL;
  ret = FLS110_Reg_Write(FLS110_REG_BASIS, &basis, 1);
  //printf("Set FLS110 basis %d status:%d\n", basis, ret);
  if (ret != HAL_OK){
    printf("Set FLS110 basis %d failed\n", basis);
    return FLS_ERROR;
  }

  HAL_Delay(10);
  ret = FLS110_Reg_Read(FLS110_REG_BASIS, &readout_basis, 1);
  //printf("Readout FLS110 basis %d status: %d\n", readout_basis, ret);
  if((ret != HAL_OK) || (readout_basis != basis))
  {
    printf("Verify FLS110 basis %d failed, readout: %d\n", basis, readout_basis);
    return FLS_ERROR;
  }
  printf("Set FLS110 basis %d success\n", basis);
  return FLS_SUCCESS;
}

static fls_ready_t FLS110_Get_Ready(void)
{
  fls_ready_t regReady = FLS_READY_ILLEGAL;
  HAL_StatusTypeDef ret;

  ret = FLS110_Reg_Read(FLS110_REG_READY, &regReady, 1);
  UNUSED(ret);//printf("Reg READY(%02X): %02X\n", ret, regReady);
  return regReady;
}

void FLS110_Sensor_Start(void)
{
  FLS110_Set_Mode(FLS_MODE_CONTINUOUS);
}

void FLS110_Sensor_Stop(void)
{
  FLS110_Set_Mode(FLS_MODE_IDLE);
}

void FLS110_Sensor_Init(void)
{
  FLS110_Check_FW_ID();
  FLS110_Check_Unique_ID();
  FLS110_Set_Avg(8);
  FLS110_Set_Basis(FLS_BASIS_DP);
  FLS110_Sensor_Start();
}

void FLS110_Sensor_Test(void)
{
  float air_speed = 0;
  //float pflow = 0.0f;

  while(FLS110_Get_Ready() != FLS_READY){
    //HAL_Delay(1);
  }
  FLS110_data_read(&air_speed);
  //FLS110_pflow_read(&pflow);
}
