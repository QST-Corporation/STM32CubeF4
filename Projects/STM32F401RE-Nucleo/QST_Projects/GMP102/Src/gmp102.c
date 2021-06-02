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
 * @file    gmp102.c
 * @author  QST AE team
 * @version V0.1
 * @date    2021-04-13
 * @id      $Id$
 * @brief   This file provides the functions for GMP102 sensor evaluation.
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
#include "gmp102.h"
#include "pSensor_util.h"

/******************************************************
 *                      Macros
 ******************************************************/
//#define __weak __attribute__((weak))
#define I2C_SLAVE_ADDR                GMP102_7BIT_I2C_ADDR
#define I2C_SLAVE_ADDR_READ           ((I2C_SLAVE_ADDR<<1)|1)
#define I2C_SLAVE_ADDR_WRITE          ((I2C_SLAVE_ADDR<<1)|0)
#define GMP102_TIMEOUT                0xFF //I2C operation timout in ms
#define GMP_MAX_PAYLOAD               20 //12U
#define FLOAT_SUPPORT

extern void Error_Handler(void);

/******************************************************
 *                 Global Variables
 ******************************************************/
I2C_HandleTypeDef hi2c1;
float fCalibParam[GMP102_CALIBRATION_PARAMETER_COUNT], fT_Celsius, fP_Pa, fAlt_m;


/******************************************************
 *                 Static Variables
 ******************************************************/
static const float GMP102_CALIB_SCALE_FACTOR[] = {
  1.0E+00,
  1.0E-05,
  1.0E-10,
  1.0E-05,
  1.0E-10,
  1.0E-15,
  1.0E-12,
  1.0E-17,
  1.0E-21 };

static const int32_t GMP102_POWER_SCALE[] = {1, 10, 100, 1000};

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

static HAL_StatusTypeDef GMP_I2C_Read(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;
  //Status = HAL_I2C_Mem_Read_2(&hi2c1, I2C_SLAVE_ADDR_READ, pData, size, GMP102_TIMEOUT);
  Status = HAL_I2C_Master_Receive(&hi2c1, I2C_SLAVE_ADDR_READ, pData, size, GMP102_TIMEOUT);
  return Status;
}

HAL_StatusTypeDef GMP_I2C_Write(uint8_t *pData, uint16_t size)
{
  HAL_StatusTypeDef Status;

  Status = HAL_I2C_Master_Transmit(&hi2c1, I2C_SLAVE_ADDR_WRITE, pData, size, GMP102_TIMEOUT);
  return Status;
}

/*!
 * @brief Read multiple data from the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values read
 * @param u8Len Number of bytes to read
 * 
 * @return Result from the burst read function
 * @retval >= 0 Success
 * @retval -127 Error null bus
 * @retval < 0  Communication error
 *
 */
HAL_StatusTypeDef gmp102_burst_read(uint8_t u8Addr, uint8_t* pu8Data, uint8_t u8Len)
{
  HAL_StatusTypeDef Status;

  Status = GMP_I2C_Write(&u8Addr, 1);
  //printf("write Reg_addr[0x%02X] status: 0x%02X\n", u8Addr, Status);
  if (Status != HAL_OK){
    return Status;
  }

  return GMP_I2C_Read(pu8Data, u8Len);
}

/*!
 * @brief Write multiple data to the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values to write
 * @param u8Len Number of bytes to write
 * 
 * @return Result from the burst write function
 * @retval >= 0 Success
 * @retval -127 Error null bus
 * @retval < 0   Communication error
 *
 */
HAL_StatusTypeDef gmp102_burst_write(uint8_t u8Addr, uint8_t* pu8Data, uint8_t u8Len)
{
  uint8_t payload[GMP_MAX_PAYLOAD] = {0,};

  payload[0] = u8Addr;
  memmove(&payload[1], pu8Data, u8Len);
  return GMP_I2C_Write(payload, u8Len+1);
}

/*!
 * @brief GMP102 initialize communication bus
 *
 * @param pbus Pointer to the I2C/SPI read/write bus support struct
 * 
 * @return Result from bus communication function
 * @retval 0 Success
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int8_t gmp102_get_pid(void){
	
  int8_t comRslt = -1;
  uint8_t u8Data;
		
  //Read chip ID
  comRslt = gmp102_burst_read(GMP102_REG_PID, &u8Data, 1);

  printf("Pid: 0x%02X, status: %d\n", u8Data, comRslt);
  return comRslt;
}
 
/*!
 * @brief GMP102 soft reset
 *
 * @param None
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int8_t gmp102_soft_reset(void){
	
  int8_t comRslt = -1;
  uint8_t u8Data = GMP102_SW_RST_SET_VALUE;
	
  //Set 00h = 0x24
  comRslt = gmp102_burst_write(GMP102_RST__REG, &u8Data, 1);

  //默认成功，不判断返回值
  return 1;
}

/*!
 * @brief Get gmp102 calibration parameters
 *        - Read calibration register AAh~BBh total 18 bytes 
 *        - Compose 9 calibration parameters from the 18 bytes
 *
 * @param fCalibParam: the calibration parameter array returned to caller
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int8_t gmp102_get_calibration_param(float* fCalibParam){
	
  uint8_t u8DataBuf[GMP102_CALIBRATION_REGISTER_COUNT];
  int8_t comRslt;
  int32_t tmp, shift, i;
	
  //read the calibration registers
  comRslt = gmp102_burst_read(GMP102_REG_CALIB00, u8DataBuf, GMP102_CALIBRATION_REGISTER_COUNT);
	
  //if(comRslt < GMP102_CALIBRATION_REGISTER_COUNT){
  //  comRslt = -1;
  //  goto EXIT;
  //}
	
  // Get the parameters
  shift = sizeof(int32_t)*8 - 16;
  for(i = 0; i < GMP102_CALIBRATION_PARAMETER_COUNT; ++i){
    tmp = (u8DataBuf[2 * i] << 8) + u8DataBuf[2 * i + 1];
    fCalibParam[i] = ((tmp << shift) >> (shift + 2)) * GMP102_POWER_SCALE[(u8DataBuf[2 * i + 1] & 0x03)] * GMP102_CALIB_SCALE_FACTOR[i];
    //printf("fCalibParam[%d]=%.2f\n", i, fCalibParam[i]);
  }
	
 EXIT:
  return comRslt;
}

/*!
 * @brief Get gmp102 calibration parameters for fixed-point compensation
 *        - Read calibration register AAh~BBh total 18 bytes
 *        - Return 9 calibration parameters with fixed-point value and power parts
 *
 * @param s16Value[]: array of the value part of the calibration parameter
 * @param u8Power[]: array of the power part of the calibration parameter
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int8_t gmp102_get_calibration_param_fixed_point(int16_t s16Value[], uint8_t u8Power[]){

  uint8_t u8DataBuf[GMP102_CALIBRATION_REGISTER_COUNT];
  int8_t comRslt;
  int16_t tmp, i;
	
  //read the calibration registers
  comRslt = gmp102_burst_read(GMP102_REG_CALIB00, u8DataBuf, GMP102_CALIBRATION_REGISTER_COUNT);
	
  if(comRslt < GMP102_CALIBRATION_REGISTER_COUNT){
    comRslt = -1;
    goto EXIT;
  }

  for(i = 0; i < GMP102_CALIBRATION_PARAMETER_COUNT; ++i){
    tmp = (u8DataBuf[2 * i] << 8) + u8DataBuf[2 * i + 1];
    s16Value[i] = (tmp>>2);
    u8Power[i] = (tmp & 0x03);
  }

 EXIT:
  return comRslt;
}

/*!
 * @brief gmp102 initialization
 *        Set AAh ~ ADh to 0x00
 *
 * @param None
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int8_t gmp102_initialization(void){
	
  int8_t comRslt = 0, s8Tmp;
  uint8_t u8Data[] = {0, 0, 0, 0};
	
  //Set AAh ~ AD to 0x00
  s8Tmp = gmp102_burst_write(GMP102_REG_CALIB00, u8Data, 4);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
	
}

/*!
 * @brief gmp102 measure temperature
 *
 * @param *ps16T calibrated temperature code returned to caller
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int8_t gmp102_measure_T(int16_t* ps16T){
	
  int8_t comRslt = 0, s8Tmp,s8timecnt = 0;
  uint8_t u8Data[2];
	
  // Set A5h = 0x00, Calibrated data out
  u8Data[0] = 0x00;
  s8Tmp = gmp102_burst_write(GMP102_REG_CONFIG1, u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
  // Set 30h = 0x08, T-Forced mode
  u8Data[0] = 0x08;
  s8Tmp = gmp102_burst_write(GMP102_REG_CMD, u8Data, 1);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
  // Wait for 02h[0] DRDY bit set
  do{

    //wait a while
	   HAL_Delay(1);
		
      s8Tmp = gmp102_burst_read(GMP102_REG_STATUS, u8Data, 1);

    if(s8Tmp < 0){ //communication error
      comRslt = s8Tmp;
      goto EXIT;
    }
    comRslt += s8Tmp;		
		
  } while(( GMP102_GET_BITSLICE(u8Data[0], GMP102_DRDY) != 1)&&(s8timecnt++ < 100));
	
  // Read 09h~0Ah
  s8Tmp = gmp102_burst_read(GMP102_REG_TEMPH, u8Data, 2);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	
	
  // Get the calibrated temperature in code
  *ps16T = (u8Data[0] << 8) + u8Data[1];
printf("gmp102_measure_T, u8Data[0]= 0x%x,u8Data[1]= 0x%x\n",u8Data[0],u8Data[1]);	
 EXIT:
  return comRslt;
}

/*!
 * @brief gmp102 measure pressure
 *
 * @param *ps32P raw pressure in code returned to caller
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int8_t gmp102_measure_P(int32_t* ps32P){

  int8_t comRslt = 0, s8Tmp,s8timecnt = 0;
  uint8_t u8Data[3];
	
  // Set A5h = 0x02, raw data out
  u8Data[0] = 0x02;
  s8Tmp = gmp102_burst_write(GMP102_REG_CONFIG1, u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
  // Set 30h = 0x09, P-Forced mode
  u8Data[0] = 0x09;
  s8Tmp = gmp102_burst_write(GMP102_REG_CMD, u8Data, 1);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
  // Wait for 02h[0] DRDY bit set
  do{

    //wait a while
	  HAL_Delay(1);
		
      s8Tmp = gmp102_burst_read(GMP102_REG_STATUS, u8Data, 1);

    if(s8Tmp < 0){ //communication error
      comRslt = s8Tmp;
      goto EXIT;
    }
    comRslt += s8Tmp;		
		
  } while(( GMP102_GET_BITSLICE(u8Data[0], GMP102_DRDY) != 1)&&(s8timecnt++ < 100));
	
  // Read 06h~08h
  s8Tmp = gmp102_burst_read(GMP102_REG_PRESSH, u8Data, 3);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	
	
  s8Tmp = sizeof(*ps32P)*8 - 24;
  // Get the raw pressure in code
  *ps32P = (u8Data[0] << 16) + (u8Data[1] << 8) + u8Data[2];
  *ps32P = (*ps32P << s8Tmp) >> s8Tmp; //24 bit sign extension
	
 EXIT:
  return comRslt;
}

/*!
 * @brief gmp102 measure pressure and temperature
 *        Read pressure first then commit pressure data conversion for the next call
 *        
 * @param *ps32P raw pressure in code returned to caller
 * @param *ps16T calibrated temperature code returned to caller
 * @param s8WaitPDrdy 1: P wait for DRDY bit set, 0: P no wait
 *
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int8_t gmp102_measure_P_T(int32_t* ps32P, int16_t* ps16T, int8_t s8PWaitDrdy){

  int8_t comRslt = 0, s8Tmp,s8timecnt = 0;
  uint8_t u8Data[3];
	
  /*
   *
   * Read raw P code
   *
   */
  if(s8PWaitDrdy){
    // Wait for 02h[0] DRDY bit set if s8PWaitDrdy is 1
    do{

      //wait a while
      HAL_Delay(1);
		
      s8Tmp = gmp102_burst_read(GMP102_REG_STATUS, u8Data, 1);

      if(s8Tmp < 0){ //communication error
	comRslt = s8Tmp;
	goto EXIT;
      }
      comRslt += s8Tmp;		
		
	} while(( GMP102_GET_BITSLICE(u8Data[0], GMP102_DRDY) != 1)&&(s8timecnt++ < 100));
  }
	
  // Read 06h~08h
  s8Tmp = gmp102_burst_read(GMP102_REG_PRESSH, u8Data, 3);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	
	
  s8Tmp = sizeof(*ps32P)*8 - 24;
  // Get the raw pressure in code
  *ps32P = (u8Data[0] << 16) + (u8Data[1] << 8) + u8Data[2];
  *ps32P = (*ps32P << s8Tmp) >> s8Tmp; //24 bit sign extension
	
  /*
   *
   * Measure calibrated T code
   *
   */
  // Set A5h = 0x00, Calibrated data out
  u8Data[0] = 0x00;
  s8Tmp = gmp102_burst_write(GMP102_REG_CONFIG1, u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
  // Set 30h = 0x08, T-Forced mode
  u8Data[0] = 0x08;
  s8Tmp = gmp102_burst_write(GMP102_REG_CMD, u8Data, 1);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
  s8timecnt	= 0;
  // Wait for 02h[0] DRDY bit set
  do{

    //wait a while
    //HAL_Delay(1);

    s8Tmp = gmp102_burst_read(GMP102_REG_STATUS, u8Data, 1);

    if(s8Tmp < 0){ //communication error
      comRslt = s8Tmp;
      goto EXIT;
    }
    comRslt += s8Tmp;		
		
  } while(( GMP102_GET_BITSLICE(u8Data[0], GMP102_DRDY) != 1)&&(s8timecnt++ < 100));
	
  // Read 09h~0Ah
  s8Tmp = gmp102_burst_read(GMP102_REG_TEMPH, u8Data, 2);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	
	
  // Get the calibrated temperature in code
  *ps16T = (u8Data[0] << 8) + u8Data[1];
	
  /*
   *
   * Commit the next pressure conversion
   *
   */
  // Set A5h = 0x02, raw data out
  u8Data[0] = 0x02;
  s8Tmp = gmp102_burst_write(GMP102_REG_CONFIG1, u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
  // Set 30h = 0x09, P-Forced mode
  u8Data[0] = 0x09;
  s8Tmp = gmp102_burst_write(GMP102_REG_CMD, u8Data, 1);

  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
}

/*!
 * @brief gmp102 temperature and pressure compensation
 *
 * @param s16T calibrated temperature in code
 * @param s32P raw pressure in code
 * @param fParam[] pressure calibration parameters
 * @param *pfT_Celsius calibrated temperature in Celsius returned to caller
 * @param *pfP_Pa calibraated pressure in Pa returned to caller
 * 
 * @return None
 *
 */
void gmp102_compensation(int16_t s16T, int32_t s32P, float fParam[], float* pfT_Celsius, float* pfP_Pa){
	
  *pfT_Celsius = GMP102_T_CODE_TO_CELSIUS(s16T);
	
  *pfP_Pa = \
    fParam[0] + \
    fParam[1]*s16T + \
    fParam[2]*s16T*s16T + \
    fParam[3]*s32P + \
    fParam[4]*s16T*s32P + \
    fParam[5]*s16T*s16T*s32P + \
    fParam[6]*s32P*s32P + \
    fParam[7]*s16T*s32P*s32P + \
    fParam[8]*s16T*s16T*s32P*s32P;
	
}

#define ShiftRight(v, s) (((v)+(1<<((s)-1)))>>(s))
#define RoundDivide(v, d) (((v)+((d)/2))/(d))

/*!
 * @brief gmp102 temperature and pressure compensation, int64_t fixed point operation
 *
 * @param s16T raw temperature in code
 * @param s32P raw pressure in code
 * @param s16Value[]: array of the value part of the calibration parameter
 * @param u8Power[]: array of the power part of the calibration parameter
 * @param *ps32T_Celsius calibrated temperature in 1/256*Celsius returned to caller
 * @param *ps32P_Pa calibrated pressure in Pa returned to caller
 * 
 * @return None
 *
 */
void gmp102_compensation_fixed_point_s64(int16_t s16T, int32_t s32P, int16_t s16Value[], uint8_t u8Power[], int32_t* ps32T_Celsius, int32_t* ps32P_Pa){

  int64_t tmp, val, s64T, s64P;
  s64T = s16T;
  s64P = s32P;

  //Temperature
  *ps32T_Celsius = s16T;

  //Pressure
  val = 0;
  //beta0
  tmp = s16Value[0] * GMP102_POWER_SCALE[u8Power[0]] * 10;
  val += tmp;
  //beta1*T
  tmp = s64T * s16Value[1];
  tmp = tmp * GMP102_POWER_SCALE[u8Power[1]];
  tmp = RoundDivide(tmp, 10000);
  val += tmp;
  //beta2*T*T
  tmp = s64T * s16Value[2];
  tmp = tmp * s64T;
  tmp = tmp * GMP102_POWER_SCALE[u8Power[2]];
  tmp = RoundDivide(tmp, 1000000000);
  val += tmp;
  //beta3*P
  tmp = s64P * s16Value[3];
  tmp = tmp * GMP102_POWER_SCALE[u8Power[3]];
  tmp = RoundDivide(tmp, 10000);
  val += tmp;
  //beta4*P*T
  tmp = s64P * s16Value[4];
  tmp = tmp * s64T;
  tmp = tmp * GMP102_POWER_SCALE[u8Power[4]];
  tmp = RoundDivide(tmp, 1000000000);
  val += tmp;
  //beta5*P*T*T
  tmp = s64P * s16Value[5];
  tmp = tmp * s64T;
  tmp = ShiftRight(tmp, 10) * s64T;
  tmp = ShiftRight(tmp, 10) * GMP102_POWER_SCALE[u8Power[5]];
  tmp = RoundDivide(tmp, 95367432);
  val += tmp;
  //beta6*P*P
  tmp = s64P * s16Value[6];
  tmp = tmp * s64P;
  tmp = ShiftRight(tmp, 7) * GMP102_POWER_SCALE[u8Power[6]];
  tmp = RoundDivide(tmp, 781250000);
  val += tmp;
  //beta7*P*P*T
  tmp = s64P * s16Value[7];
  tmp = tmp * s64P;
  tmp = ShiftRight(tmp, 10) * s64T;
  tmp = ShiftRight(tmp, 10) * GMP102_POWER_SCALE[u8Power[7]];
  tmp = RoundDivide(tmp, 9536743164);
  val += tmp;
  //beta8*P*P*T*T
  tmp = s64P * s16Value[8];
  tmp = tmp * s64P;
  tmp = ShiftRight(tmp, 9) * ShiftRight(s64T, 1);
  tmp = ShiftRight(tmp, 12) * ShiftRight(s64T, 3);
  tmp = ShiftRight(tmp, 7) * GMP102_POWER_SCALE[u8Power[8]];
  tmp = RoundDivide(tmp, 23283064365);
  val += tmp;

  *ps32P_Pa = (int32_t)RoundDivide(val, 10);

  return;
}

/*!
 * @brief gmp102 set pressure OSR
 *
 * @param osrP OSR to set
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int8_t gmp102_set_P_OSR(GMP102_P_OSR_Type osrP){
	
  int8_t comRslt = 0, s8Tmp;
  uint8_t u8Data;
	
  //Read A6h
  s8Tmp = gmp102_burst_read(GMP102_REG_CONFIG2, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	

  //Set the A6h[2:0] OSR bits
  u8Data = GMP102_SET_BITSLICE(u8Data, GMP102_P_OSR, osrP);
  s8Tmp = gmp102_burst_write(GMP102_REG_CONFIG2, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
}


/*!
 * @brief gmp102 set temperature OSR
 *
 * @param osrT OSR to set
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int8_t gmp102_set_T_OSR(GMP102_T_OSR_Type osrT){
	
  int8_t comRslt = 0, s8Tmp;
  uint8_t u8Data;
	
  //Read A7h
  s8Tmp = gmp102_burst_read(GMP102_REG_CONFIG3, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;	

  //Set the A7h[2:0] OSR bits
  u8Data = GMP102_SET_BITSLICE(u8Data, GMP102_T_OSR, osrT);
  s8Tmp = gmp102_burst_write(GMP102_REG_CONFIG3, &u8Data, 1);
	
  if(s8Tmp < 0){ //communication error
    comRslt = s8Tmp;
    goto EXIT;
  }
  comRslt += s8Tmp;
	
 EXIT:
  return comRslt;
}

void gmp102_init(void)
{
  int8_t s8Res;

  gmp102_get_pid(); //Initailze GMP102

  /* GMP102 soft reset */
  s8Res = gmp102_soft_reset();

  /* Wait 100ms for reset complete */
  HAL_Delay(100);

  /* GMP102 get the pressure calibration parameters */
#ifdef FLOAT_SUPPORT

  s8Res = gmp102_get_calibration_param(fCalibParam);
#else
  s8Res = gmp102_get_calibration_param_fixed_point(s16Value, u8Power);

#endif	
  /* GMP102 initialization setup */
  s8Res = gmp102_initialization();

  //设置采样时间，缩短单次转化时间
  //gmp102_set_P_OSR(GMP102_P_OSR_256);//1.54ms
  //gmp102_set_T_OSR(GMP102_T_OSR_256);//1.54ms

  /* set sea level reference pressure */
  //If not set, use default 101325 Pa for pressure altitude calculation
  set_sea_level_pressure_base(101325.f);//100360.f

}


void GMP102_Sensor_Test(void)
{
  int8_t s8Res;
  int16_t s16Value[GMP102_CALIBRATION_PARAMETER_COUNT];
  uint8_t u8Power[GMP102_CALIBRATION_PARAMETER_COUNT];
  int16_t s16T;
  int32_t s32P, s32P_Pa, s32T_Celsius;

  /* Measure P */
  s8Res = gmp102_measure_P(&s32P);
  printf("P(code)=%d\r", s32P);

  /* Mesaure T */
  s8Res = gmp102_measure_T(&s16T);
  printf("T(code)=%d\r", s16T);

  /* Compensation */
#ifdef FLOAT_SUPPORT

  gmp102_compensation(s16T, s32P, fCalibParam, &fT_Celsius, &fP_Pa);
#else
  gmp102_compensation_fixed_point_s64(s16T, s32P, s16Value, u8Power, &s32T_Celsius, &s32P_Pa);

#endif
  printf("P(Pa)=%f, %d\r", fP_Pa, s32P);
  printf("T(C)=%f, %f\r", fT_Celsius, s32T_Celsius/256.0);

  /* Pressure Altitude */
  fAlt_m = pressure2Alt(fP_Pa);
  printf("Alt(m)=%f\r", fAlt_m);
}
