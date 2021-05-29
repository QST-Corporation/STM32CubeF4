/*******************************************************************************
* Copyright (C), 2000-2016,  Electronic Technology Co., Ltd.
* FileName: SPL06_03.c
* Author: Li.wen
* Version:
* Date: 2015-11-30     
* Description:Define the operation interface of SPL06-01
* History:
*    1. Date: 2015-11-30
*       Author: Li.wen
*       Modification: create
*    2. Date：2016-10-12
		Author：lan.xing
		Modification：modified the calibrate coefficient
* Others:           
*******************************************************************************/
#ifndef SPL06_01_C
#define SPL06_01_C

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "bsp_uart.h"
#include "spl06_01.h"

extern I2C_HandleTypeDef hi2c1;
static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;

HAL_StatusTypeDef spl0601_write(uint8_t hwadr, uint8_t regadr, uint8_t val);
uint8_t spl0601_read(uint8_t hwadr, uint8_t regadr);
void spl0601_get_calib_param(void);

static HAL_StatusTypeDef SPL_I2C_Read(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;
  Status = HAL_I2C_Master_Receive(&hi2c1, SPL_SLAVE_ADDR_READ, pData, size, SPL_TIMEOUT);
  return Status;
}

HAL_StatusTypeDef SPL_I2C_Write(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;

  Status = HAL_I2C_Master_Transmit(&hi2c1, SPL_SLAVE_ADDR_WRITE, pData, size, SPL_TIMEOUT);
  return Status;
}

/*****************************************************************************
 Function: spl0601_write
 Description: this function will write data to specofic register through software I2C bus
 Input:  uint8_t hwadr   hardware I2C address
         uint8_t regadr  register address
         uint8_t val     write-in value          
 Output: 
 Return: 
 Calls: 
 Called By: 
*****************************************************************************/
HAL_StatusTypeDef spl0601_write(uint8_t hwadr, uint8_t regadr, uint8_t val)
{
    uint8_t payload[2] = {0,};

    payload[0] = regadr;
    payload[1] = val;
    return SPL_I2C_Write(payload, sizeof(payload));
}

/*****************************************************************************
 Function: spl0601_read
 Description: this function will read register data through software I2C bus
 Input: uint8_t hwadr   hardware I2C address
        uint8_t regadr  register address        
 Output: 
 Return: uint8_t readout value
 Calls: 
 Called By: 
*****************************************************************************/
uint8_t spl0601_read(uint8_t hwadr, uint8_t regadr)
{
    uint8_t val = 0;
    SPL_I2C_Write(&regadr, 1);
    SPL_I2C_Read(&val, 1);
    return val;
}

void spl0601_read_multi_bytes(uint8_t hwadr, uint8_t regadr, uint8_t *pdata, uint8_t size)
{
    SPL_I2C_Write(&regadr, 1);
    SPL_I2C_Read(pdata, size);
}

/*****************************************************************************
 Function: spl0601_init
 Description: initialization
 Input: void             
 Output: 
 Return: void 
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_init(void)
{
    p_spl0601 = &spl0601; /* read Chip Id */
    p_spl0601->i32rawPressure = 0;
    p_spl0601->i32rawTemperature = 0;
    p_spl0601->chip_id = 0x34;
    spl0601_get_calib_param();
    // sampling rate = 1Hz; Pressure oversample = 2;
    spl0601_rateset(PRESSURE_SENSOR,32, 8);   
    // sampling rate = 1Hz; Temperature oversample = 1; 
    spl0601_rateset(TEMPERATURE_SENSOR,32, 8);
    //Start background measurement
    
}

/*****************************************************************************
 Function: spl0601_rateset
 Description: set sample rate and over sample rate per second for specific sensor
 Input:     uint8_t u8OverSmpl  oversample rate         Maximal = 128
            uint8_t u8SmplRate  sample rate(Hz) Maximal = 128
            uint8_t iSensor     0: Pressure; 1: Temperature 
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_rateset(uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    uint8_t reg = 0;
    int32_t i32kPkT = 0;
    switch(u8SmplRate)
    {
        case 2:
            reg |= (1<<4);
            break;
        case 4:
            reg |= (2<<4);
            break;
        case 8:
            reg |= (3<<4);
            break;
        case 16:
            reg |= (4<<4);
            break;
        case 32:
            reg |= (5<<4);
            break;
        case 64:
            reg |= (6<<4);
            break;
        case 128:
            reg |= (7<<4);
            break;
        case 1:
        default:
            break;
    }
    switch(u8OverSmpl)
    {
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if(iSensor == PRESSURE_SENSOR)
    {
        p_spl0601->i32kP = i32kPkT;
        spl0601_write(HW_ADR, 0x06, reg);
        if(u8OverSmpl > 8)
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg | 0x04);
        }
        else
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg & (~0x04));
        }
    }
    if(iSensor == TEMPERATURE_SENSOR)
    {
        p_spl0601->i32kT = i32kPkT;
        spl0601_write(HW_ADR, 0x07, reg|0x80);  //Using mems temperature
        if(u8OverSmpl > 8)
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg | 0x08);
        }
        else
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg & (~0x08));
        }
    }

}

/*****************************************************************************
 Function: spl0601_get_calib_param
 Description: obtain the calibrated coefficient
 Input: void     
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_get_calib_param(void)
{
    uint32_t h;
    uint32_t m;
    uint32_t l;
    h =  spl0601_read(HW_ADR, 0x10);
    l  =  spl0601_read(HW_ADR, 0x11);
    p_spl0601->calib_param.c0 = (int16_t)h<<4 | l>>4;
    p_spl0601->calib_param.c0 = (p_spl0601->calib_param.c0&0x0800)?(0xF000|p_spl0601->calib_param.c0):p_spl0601->calib_param.c0;
    h =  spl0601_read(HW_ADR, 0x11);
    l  =  spl0601_read(HW_ADR, 0x12);
    p_spl0601->calib_param.c1 = (int16_t)(h&0x0F)<<8 | l;
    p_spl0601->calib_param.c1 = (p_spl0601->calib_param.c1&0x0800)?(0xF000|p_spl0601->calib_param.c1):p_spl0601->calib_param.c1;
    h =  spl0601_read(HW_ADR, 0x13);
    m =  spl0601_read(HW_ADR, 0x14);
    l =  spl0601_read(HW_ADR, 0x15);
    p_spl0601->calib_param.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    p_spl0601->calib_param.c00 = (p_spl0601->calib_param.c00&0x080000)?(0xFFF00000|p_spl0601->calib_param.c00):p_spl0601->calib_param.c00;
    h =  spl0601_read(HW_ADR, 0x15);
    m =  spl0601_read(HW_ADR, 0x16);
    l =  spl0601_read(HW_ADR, 0x17);
    p_spl0601->calib_param.c10 = (int32_t)(h&0x0F)<<16 | (int32_t)m<<8 | l;
    p_spl0601->calib_param.c10 = (p_spl0601->calib_param.c10&0x080000)?(0xFFF00000|p_spl0601->calib_param.c10):p_spl0601->calib_param.c10;
    h =  spl0601_read(HW_ADR, 0x18);
    l  =  spl0601_read(HW_ADR, 0x19);
    p_spl0601->calib_param.c01 = (int16_t)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x1A);
    l  =  spl0601_read(HW_ADR, 0x1B);
    p_spl0601->calib_param.c11 = (int16_t)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x1C);
    l  =  spl0601_read(HW_ADR, 0x1D);
    p_spl0601->calib_param.c20 = (int16_t)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x1E);
    l  =  spl0601_read(HW_ADR, 0x1F);
    p_spl0601->calib_param.c21 = (int16_t)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x20);
    l  =  spl0601_read(HW_ADR, 0x21);
    p_spl0601->calib_param.c30 = (int16_t)h<<8 | l;
}

/*****************************************************************************
 Function: spl0601_start_temperature
 Description: start one measurement for temperature
 Input: void    
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_start_temperature(void)
{
    spl0601_write(HW_ADR, 0x08, 0x02);
}

/*****************************************************************************
 Function: spl0601_start_pressure
 Description: start one measurement for pressure
 Input: void       
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/

void spl0601_start_pressure(void)
{
    spl0601_write(HW_ADR, 0x08, 0x01);
}
/*****************************************************************************
 Function: spl0601_start_continuous
 Description: Select mode for the continuously measurement
 Input: uint8_t mode  1: pressure; 2: temperature; 3: pressure and temperature        
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_start_continuous(uint8_t mode)
{
    spl0601_write(HW_ADR, 0x08, mode+4);
}

void spl0601_stop(void)
{
    spl0601_write(HW_ADR, 0x08, 0);
}

/*****************************************************************************
 Function: spl0601_get_raw_temp
 Description:obtain the original temperature value and turn them into 32bits-integer 
 Input: void          
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_get_raw_temp(void)
{
    uint8_t h,m,l;
    uint8_t regVal[3] = {0};
    spl0601_read_multi_bytes(HW_ADR, 0x03, regVal, sizeof(regVal));
    h = regVal[0];
    m = regVal[1];
    l = regVal[2];

    p_spl0601->i32rawTemperature = (int32_t)h<<16 | (int32_t)m<<8 | (int32_t)l;
    p_spl0601->i32rawTemperature= (p_spl0601->i32rawTemperature&0x800000) ? (0xFF000000|p_spl0601->i32rawTemperature) : p_spl0601->i32rawTemperature;
}

/*****************************************************************************
 Function: spl0601_get_raw_pressure
 Description: obtain the original pressure value and turn them into 32bits-integer
 Input: void       
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_get_raw_pressure(void)
{
    uint8_t h,m,l;
    uint8_t regVal[3] = {0};
    spl0601_read_multi_bytes(HW_ADR, 0x00, regVal, sizeof(regVal));
    h = regVal[0];
    m = regVal[1];
    l = regVal[2];

    p_spl0601->i32rawPressure = (int32_t)h<<16 | (int32_t)m<<8 | (int32_t)l;
    p_spl0601->i32rawPressure= (p_spl0601->i32rawPressure&0x800000) ? (0xFF000000|p_spl0601->i32rawPressure) : p_spl0601->i32rawPressure;
}

/*****************************************************************************
 Function: spl0601_get_temperature
 Description:  return calibrated temperature value base on original value.
 Input: void          
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
float spl0601_get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
    return fTCompensate;
}

/*****************************************************************************
 Function: spl0601_get_pressure
 Description: return calibrated pressure value base on original value.
 Input: void            
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/

float spl0601_get_pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fPsc = p_spl0601->i32rawPressure / (float)p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * (p_spl0601->calib_param.c20 + fPsc* p_spl0601->calib_param.c30);
    qua3 = fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);

    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}

void spl0601_init_and_start(void)
{
    spl0601_init();
    spl0601_start_continuous(CONTINUOUS_P_AND_T);
}

void spl0601_update_pressure(float *pPress, uint32_t *pTime)
{
    float splTemp = 0.0f;
    float splPressure = 0.0f;

    if ((pPress == NULL) || (pTime == NULL)) {
      return;
    }

    spl0601_get_raw_temp();
    spl0601_get_raw_pressure();
    splTemp = spl0601_get_temperature();
    splPressure = spl0601_get_pressure();
    *pPress = splPressure;
    *pTime = HAL_GetTick();
    printf("%ld: SPLtemp %.1f, pressure %.2f\n", *pTime, splTemp, splPressure);
}

#endif
