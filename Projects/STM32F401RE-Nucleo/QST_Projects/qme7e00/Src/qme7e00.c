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
 * @file    qme7e00.c
 * @author  QST AE team
 * @version V0.1
 * @date    2021-04-13
 * @id      $Id$
 * @brief   This file provides the functions for qme7e00 sensor evaluation.
 *          |-----------------------|
 *          |--F401RE---|--qme7e00--|
 *          |-----------------------|
 *          |---PB6-----|----SCL----|
 *          |---PB7-----|----SDA----|
 *          |---+3.3V---|----VDD----|
 * @note
 *
 *****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "bsp_uart.h"
#include "qme7e00.h"
#include "math.h"

/******************************************************
 *                      Macros
 ******************************************************/
//#define __weak __attribute__((weak))
#define I2C_SLAVE_ADDR                0x31
#define I2C_SLAVE_ADDR_READ           ((I2C_SLAVE_ADDR<<1)|1)
#define I2C_SLAVE_ADDR_WRITE          ((I2C_SLAVE_ADDR<<1)|0)
#define QME7E00_TIMEOUT_MS            0x0A //I2C operation timout in ms
#define QME7E00_MAX_PAYLOAD           0x0C //12U
#define QME7E00_REG_FW_ID             0x10
#define QME7E00_REG_UNIQUE_ID         0x11
#define QME7E00_REG_FW_BUILD          0x12
#define QME7E00_REG_FW_RELEASE        0x13
#define QME7E00_REG_AVG_WINDOW        0x20
#define QME7E00_REG_MODE              0x21
#define QME7E00_REG_READY             0x2F
#define QME7E00_REG_READING           0x40
#define QME7E00_REG_SENSOR_H          0x41
#define QME7E00_REG_SENSOR_PWR        0x42
#define QME7E00_REG_FLOW_TEMP         0x43
#define QME7E00_REG_P_FLOW            0xE0
#define QME7E00_REG_RH_FLOW           0xE1
#define QME7E00_REG_CAL_TEMP          0xFC
#define QME7E00_REG_CAL0              0xF0
#define QME7E00_REG_CAL1              0xF1
#define QME7E00_REG_CAL2              0xF2
#define QME7E00_REG_CAL3              0xF3
#define QME7E00_REG_CALIBRATE         0xFF
#define QME7E00_REG_C1                0xC1
#define QME7E00_REG_C2                0xC2
#define QME7E00_REG_C3                0xC3
#define QME7E00_REG_BASIS             0xCB
#define QME7E00_FW_ID                 0x0F100A

typedef enum {
  QME_MODE_IDLE,
  QME_MODE_CONTINUOUS,
  QME_MODE_SINGLESHOT,
  QME_MODE_ILLEGAL = 0xFFU
}qme_mode_t;

typedef enum {
  QME_SUCCESS    = 0x00U,
  QME_ERROR      = 0x01U
}qme_status_t;

typedef enum {
  QME_NOTREADY   = 0x00U,
  QME_READY      = 0x01U,
  QME_READY_ILLEGAL = 0xFFU
}qme_ready_t;

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
void QME_I2C1_Init(void)
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

static HAL_StatusTypeDef QME_I2C_Read(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;
  //Status = HAL_I2C_Mem_Read_2(&hi2c1, I2C_SLAVE_ADDR_READ, pData, size, QME7E00_TIMEOUT_MS);
  Status = HAL_I2C_Master_Receive(&hi2c1, I2C_SLAVE_ADDR_READ, pData, size, QME7E00_TIMEOUT_MS);
  return Status;
}

static HAL_StatusTypeDef QME_I2C_Write(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;

  Status = HAL_I2C_Master_Transmit(&hi2c1, I2C_SLAVE_ADDR_WRITE, pData, size, QME7E00_TIMEOUT_MS);
  return Status;
}

static HAL_StatusTypeDef QME7E00_Reg_Read(uint8_t reg, uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;

  Status = QME_I2C_Write(&reg, 1);
  if (Status != HAL_OK) {
    return Status;
  }

  return QME_I2C_Read(pData, size);
}

static HAL_StatusTypeDef QME7E00_Reg_Write(uint8_t reg, uint8_t *pData, uint16_t size)
{
  uint8_t payload[QME7E00_MAX_PAYLOAD] = {0,};

  payload[0] = reg;
  memmove(&payload[1], pData, size);
  return QME_I2C_Write(payload, size+1);
}

static qme_status_t QME7E00_pflow_read(float *pdata)
{
  HAL_StatusTypeDef ret;
  uint8_t	regVal[4]={0};
  uint32_t regPflow = 0;
  float pflow = 0.0f;

  if(pdata == NULL)
  {
    return QME_ERROR;
  }
  ret = QME7E00_Reg_Read(QME7E00_REG_P_FLOW, regVal, 4);
  memmove((uint8_t *)&regPflow, regVal, sizeof(regVal));
  pflow = (float)regPflow/256;

  printf("QME7E00 Pflow(%d), %d, %.4fPa\n", ret, regPflow, pflow);
  *pdata = pflow;

  return ret == HAL_OK ? QME_SUCCESS : QME_ERROR;
}

static qme_status_t QME7E00_data_read(float *pdata, float *pTemp)
{
  HAL_StatusTypeDef ret;
  uint8_t	regVal[4]={0}, regTemp[2]={0};
  uint32_t reading = 0;
  float flowTemp = 0.0f;
  float dp_raw = 0.0f;
  float pressure = 0.0f;

  if(pdata == NULL)
  {
    return QME_ERROR;
  }
  ret = QME7E00_Reg_Read(QME7E00_REG_READING, regVal, 4);
  memmove((uint8_t *)&reading, regVal, sizeof(regVal));
  dp_raw = (float)reading/256;
  QME7E00_Reg_Read(QME7E00_REG_FLOW_TEMP, regTemp, sizeof(regTemp));
  flowTemp = (float)((int16_t)regTemp[1] << 8 | regTemp[0])/256;

  pressure = 4e-6f*dp_raw*dp_raw + 0.0661f*dp_raw;
  //printf("QME7E00(%d), %d, %.4f, %.1f\n", ret, reading, pressure, flowTemp);
  *pdata = pressure;
  *pTemp = flowTemp;

  return ret == HAL_OK ? QME_SUCCESS : QME_ERROR;
}

static qme_status_t QME7E00_Check_FW_ID(void)
{
  //uint32_t readoutFwId = 0;
  uint8_t regVal[4] = {0,};
  uint32_t fw_id = 0;
  HAL_StatusTypeDef ret;

  // wait for slave ready
  while ((fw_id != QME7E00_FW_ID) && (fw_id != (QME7E00_FW_ID-1))) {
    ret = QME7E00_Reg_Read(QME7E00_REG_FW_ID, regVal, sizeof(regVal));
    fw_id = (uint32_t)regVal[3]<<24 | (uint32_t)regVal[2]<<16 | (uint32_t)regVal[1]<<8 | (uint32_t)regVal[0];
    printf("FW_ID: 0x%04X\n", fw_id);
    HAL_Delay(1);
  }
  return ret == HAL_OK ? QME_SUCCESS : QME_ERROR;
}

static qme_status_t QME7E00_Check_Unique_ID(void)
{
  uint8_t regVal[12] = {0,};
  HAL_StatusTypeDef ret;

  ret = QME7E00_Reg_Read(QME7E00_REG_UNIQUE_ID, regVal, sizeof(regVal));
  printf("Processor ID: ");
  for(uint8_t i=0; i<sizeof(regVal); i++) {
    printf("%02X,", regVal[i]);
  }
  printf("\n");
  return ret == HAL_OK ? QME_SUCCESS : QME_ERROR;
}

static qme_status_t QME7E00_Set_Mode(qme_mode_t mode)
{
  HAL_StatusTypeDef ret;
  qme_mode_t readout_mode = QME_MODE_ILLEGAL;
  ret = QME7E00_Reg_Write(QME7E00_REG_MODE, &mode, 1);
  //printf("Set QME7E00 mode %d status:%d\n", mode, ret);
  if (ret != HAL_OK){
    printf("Set QME7E00 mode %d failed\n", mode);
    return QME_ERROR;
  }

  HAL_Delay(10);
  ret = QME7E00_Reg_Read(QME7E00_REG_MODE, &readout_mode, 1);
  //printf("Readout QME7E00 mode %d status: %d\n", readout_mode, ret);
  if((ret != HAL_OK) || (readout_mode != mode))
  {
    printf("Verify QME7E00 mode %d failed, readout: %d\n", mode, readout_mode);
    return QME_ERROR;
  }
  printf("Set QME7E00 mode %d success\n", mode);
  return QME_SUCCESS;
}

static qme_status_t QME7E00_Set_Avg(uint8_t avg)
{
  HAL_StatusTypeDef ret;
  uint8_t readout_avg = 0xFF;
  ret = QME7E00_Reg_Write(QME7E00_REG_AVG_WINDOW, &avg, 1);
  //printf("Set QME7E00 Avg %d status:%d\n", avg, ret);
  if (ret != HAL_OK){
    printf("Set QME7E00 Avg %d failed\n", avg);
    return QME_ERROR;
  }

  HAL_Delay(10);
  ret = QME7E00_Reg_Read(QME7E00_REG_AVG_WINDOW, &readout_avg, 1);
  //printf("Readout QME7E00 Avg %d status: %d\n", readout_avg, ret);
  if((ret != HAL_OK) || (readout_avg != avg))
  {
    printf("Verify QME7E00 Avg %d failed, readout: %d\n", avg, readout_avg);
    return QME_ERROR;
  }
  printf("Set QME7E00 Avg %d success\n", avg);
  return QME_SUCCESS;
}

static qme_ready_t QME7E00_Get_Ready(void)
{
  qme_ready_t regReady = QME_READY_ILLEGAL;
  HAL_StatusTypeDef ret;

  ret = QME7E00_Reg_Read(QME7E00_REG_READY, &regReady, 1);
  UNUSED(ret);//printf("Reg READY(%02X): %02X\n", ret, regReady);
  return regReady;
}

void QME7E00_Sensor_Start(void)
{
  QME7E00_Set_Mode(QME_MODE_CONTINUOUS);
}

void QME7E00_Sensor_Stop(void)
{
  qme_status_t ret=QME_ERROR;
  while (ret != QME_SUCCESS)
  {
    HAL_Delay(500);
    ret = QME7E00_Set_Mode(QME_MODE_IDLE);
  }
}

void QME7E00_Sensor_Init(void)
{
  QME7E00_Check_FW_ID();
  QME7E00_Sensor_Stop(); //stop sensor first if it is running.
  QME7E00_Check_Unique_ID();
  QME7E00_Set_Avg(8);
  QME7E00_Sensor_Start();
}

// 输入：压差、静压、当地温度（华氏摄氏度）
float cal_true_airspeed(float diff_pressure, float pressure_static, float tempreture_cel){
    float AIR_DENSITY_SEA_LEVEL_15CEL = 1.225f;
    float CONSTANTS_AIR_GAS_CONST = 287.1f;
    float TEMPRETURE_ZERO = -273.15f;
    float air_density = pressure_static / ( (CONSTANTS_AIR_GAS_CONST) * (tempreture_cel - TEMPRETURE_ZERO) );

    float true_airspeed =0.0f;
    if(diff_pressure > 0.0f){
        true_airspeed = sqrtf(2.0f  * diff_pressure / air_density) ;
    }else{
        true_airspeed = -sqrtf(2.0f * fabsf(diff_pressure) / air_density );
    }

    return true_airspeed;
}

// 如果没有静压，只能把当地空气密度估计为海平面15华氏度情况下的密度
// 这是简化版
float cal_true_airspeed_lite(float diff_pressure){
    float AIR_DENSITY_SEA_LEVEL_15CEL = 1.225;
    float true_airspeed = sqrtf(2.0f* diff_pressure / AIR_DENSITY_SEA_LEVEL_15CEL);
    return true_airspeed;
}

void QME7E00_Sensor_Test(void)
{
  float diff_pressure = 0;
  float temperature = 0;
  //float pflow = 0.0f;
  float pressure_static = 101325.0f;
  float tempreture_cel = 78.8f;
  float true_airspeed = 0.0f;
  float true_airspeed_lite = 0.0f;
  uint32_t freshTimestamp;

  while(QME7E00_Get_Ready() != QME_READY){
    //HAL_Delay(1); //ODR 4ms
  }
  freshTimestamp = HAL_GetTick();
  QME7E00_data_read(&diff_pressure, &temperature);
  //QME7E00_pflow_read(&pflow);
  true_airspeed = cal_true_airspeed(diff_pressure, pressure_static, tempreture_cel);

  true_airspeed_lite = cal_true_airspeed_lite(diff_pressure);

  //mm:ss:ms, diff pressure, True Airspeed, True Airspeed Lite, Air temperature
  printf("%02d:%02d:%03d DP:%.2f, TAS:%.2f, %.2f, T:%.1f\n", freshTimestamp/60000, (freshTimestamp%60000)/1000, (freshTimestamp%60000)%1000,
                 diff_pressure, true_airspeed, true_airspeed_lite, temperature);
}
