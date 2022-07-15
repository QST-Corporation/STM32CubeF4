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
 * @file    i2c_interface.c
 * @author  QST AE team
 * @version V0.1
 * @date    2022-07-08
 * @id      $Id$
 * @brief   This file provides the functions for I2C sensor evaluation.
 *          |----------------------|
 *          |--F401RE---|---I2C----|
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

#define HARDWARE_IIC      1
#define SOFTWARE_IIC      2
#define IIC_INTERFACE     SOFTWARE_IIC

#if(IIC_INTERFACE == HARDWARE_IIC)
/******************************************************
 *                      Macros
 ******************************************************/
//#define __weak __attribute__((weak))
#define I2C_DEV_TIMEOUT               0xFF //I2C operation timout in ms

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
void BSP_I2C1_Init(void)
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

HAL_StatusTypeDef BSP_I2C_Read(uint8_t i2c_dev_addr, uint8_t *pData, uint16_t size)
{
  uint8_t i2c_dev_read_addr = (i2c_dev_addr<<1)|1;
  //return HAL_I2C_Mem_Read_2(&hi2c1, i2c_dev_read_addr, pData, size, I2C_DEV_TIMEOUT);
  return HAL_I2C_Master_Receive(&hi2c1, i2c_dev_read_addr, pData, size, I2C_DEV_TIMEOUT);
}

HAL_StatusTypeDef BSP_I2C_Write(uint8_t i2c_dev_addr, uint8_t *pData, uint16_t size)
{
  uint8_t i2c_dev_write_addr = (i2c_dev_addr<<1)|0;
  return HAL_I2C_Master_Transmit(&hi2c1, i2c_dev_write_addr, pData, size, I2C_DEV_TIMEOUT);
}

#else  //SOFTWARE_IIC

#include "stm32f4xx_hal_tim.h"

  /**I2C GPIO Configuration    
  PB6     ------> I2C_SCL
  PB7     ------> I2C_SDA 
  */
#define I2C_SCL_PIN                   GPIO_PIN_6
#define I2C_SDA_PIN                   GPIO_PIN_7
#define I2C_GPIO_PORT                 GPIOB
/** SCL&SDA hardware operation  */
#define IIC_SCL_Set(x)          HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, (GPIO_PinState)x)//SCL_PIN_SET(x)
#define IIC_SDA_Set(x)          HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, (GPIO_PinState)x)//SDA_PIN_SET(x)
#define IIC_SDA_Read()          HAL_GPIO_ReadPin(I2C_GPIO_PORT, I2C_SDA_PIN)//SDA_PIN_GET()

/******** Private function declarations ***************************************/
static void bsp_iic_start(void);
static void bsp_iic_stop(void);
static void bsp_iic_SandAck(void);
static void bsp_iic_SandNack(void);
static uint8_t bsp_iic_Wait_Ack(void);
static uint8_t bsp_iic_read_byte(uint8_t ack);
static void bsp_iic_send_byte(uint8_t data);
static void config_sda_out(void);
static void config_scl_out(void);

/******** Variables ***********************************************************/
TIM_HandleTypeDef htim2;

/******** Function Prototype **************************************************/
extern void Error_Handler(void);

/*
 ** PUBLIC FUNCTION: delay_us()
 *
 *  DESCRIPTION:
 *      delay micros us.
 *
 *  PARAMETERS:
 *      micros: delay value by 1us
 *             
 *  RETURNS:
 *      None.
 */  
void delay_us(uint32_t us)
{  
  __HAL_TIM_SetCounter(&htim2, 0);
  //printf("clear counter %d:\n", __HAL_TIM_GetCounter(&htim2));

  __HAL_TIM_ENABLE(&htim2);

  while(__HAL_TIM_GetCounter(&htim2) < us)
  {
    //printf("%d\n", __HAL_TIM_GetCounter(&htim2));
  }
  //printf("excnt:%d, us %d\n", __HAL_TIM_GetCounter(&htim2), us);
  /* Disable the Peripheral */
  __HAL_TIM_DISABLE(&htim2);
}

void set_sda_pin_dir(uint32_t mode)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  GPIO_InitStruct.Pin = I2C_SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = I2C_SDA_PIN;
  GPIO_InitStruct.Mode = mode; //for input and output, need external pull-up
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);
}

/*
 ** PRIVATE FUNCTION: config_sda_in()
 *
 *  DESCRIPTION:
 *      config SDA pin for input.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
static void config_sda_in(void)
{
    /* User add related code */
    set_sda_pin_dir(GPIO_MODE_INPUT);
}

/*
 ** PRIVATE FUNCTION: config_sda_out()
 *
 *  DESCRIPTION:
 *      config SDA pin for output.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void config_sda_out(void)
{
    /* User add related code */
    set_sda_pin_dir(GPIO_MODE_OUTPUT_OD);
}

/*
 ** PRIVATE FUNCTION: config_scl_out()
 *
 *  DESCRIPTION:
 *      config scl pin for output.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void config_scl_out(void)
{
    /* User add related code */
    GPIO_InitTypeDef  GPIO_InitStruct;

    GPIO_InitStruct.Pin = I2C_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);
}

/*
 ** PRIVATE FUNCTION: bsp_iic_start()
 *
 *  DESCRIPTION:
 *      generate a i2c start or restart.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void bsp_iic_start(void)
{
    config_sda_out();
    IIC_SDA_Set(1);
    IIC_SCL_Set(1);
    delay_us(50);
    IIC_SDA_Set(0);
    delay_us(20);
    IIC_SCL_Set(0);
}

/*
 ** PRIVATE FUNCTION: bsp_iic_stop()
 *
 *  DESCRIPTION:
 *      generate a i2c stop.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void bsp_iic_stop(void)
{
    config_sda_out();
    IIC_SCL_Set(0);
    IIC_SDA_Set(0);
    delay_us(20);
    IIC_SCL_Set(1);
    delay_us(15);
    IIC_SDA_Set(1);
    delay_us(20);	
}

/*
 ** PRIVATE FUNCTION: bsp_iic_SandAck()
 *
 *  DESCRIPTION:
 *      generate a i2c ack to slave.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void bsp_iic_SandAck(void)
{
    IIC_SCL_Set(0);
    config_sda_out();
    IIC_SDA_Set(0);
    delay_us(15);
    IIC_SCL_Set(1);
    delay_us(15);
    IIC_SCL_Set(0);
}

/*
 ** PRIVATE FUNCTION: bsp_iic_SandNack()
 *
 *  DESCRIPTION:
 *      generate a i2c noack to slave.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void bsp_iic_SandNack(void)
{
    IIC_SCL_Set(0);
    config_sda_out(); 	
    IIC_SDA_Set(1);
    delay_us(15);
    IIC_SCL_Set(1);
    delay_us(15);
    IIC_SCL_Set(0);
}

/*
 ** PRIVATE FUNCTION: bsp_iic_Wait_Ack()
 *
 *  DESCRIPTION:
 *      wait a i2c ack from slave.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
uint8_t bsp_iic_Wait_Ack(void)
{
    uint8_t ucErrTime=0;

    config_sda_in();
    IIC_SCL_Set(0);
    delay_us(40);
    IIC_SCL_Set(1);

    while(IIC_SDA_Read()){
        ucErrTime++;
        if(ucErrTime>150){
            bsp_iic_stop();	
            return 1;
        }
        delay_us(10); 
    }
    
    delay_us(40);
    IIC_SCL_Set(0);
    return 0;  
}

/*
 ** PRIVATE FUNCTION: bsp_iic_read_byte()
 *
 *  DESCRIPTION:
 *      read a byte from i2c slave.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
uint8_t bsp_iic_read_byte(uint8_t ack)
{
    unsigned char i,receive=0;
    config_sda_in(); 

    for(i=0;i<8;i++){
        IIC_SCL_Set(0);
        delay_us(15);
        IIC_SCL_Set(1);
        receive<<=1;
        if(IIC_SDA_Read())
        receive++;   
        delay_us(15);	
    }
    if(!ack)
        bsp_iic_SandNack();   
    else
        bsp_iic_SandAck();  
    
    return receive;
}

/*
 ** PRIVATE FUNCTION: bsp_iic_send_byte()
 *
 *  DESCRIPTION:
 *      write a byte to i2c slave.
 *
 *  PARAMETERS:
 *      data: data to write
 *             
 *  RETURNS:
 *      None.
 */
void bsp_iic_send_byte(uint8_t data)
{
   uint8_t t;   
   config_sda_out();
   IIC_SCL_Set(0);  
    
	for(t=0;t<8;t++){              
        if(data&0x80)
          IIC_SDA_Set(1);
        else
          IIC_SDA_Set(0);
        data<<=1;
        delay_us(10);
        IIC_SCL_Set(1);
        delay_us(15);
        IIC_SCL_Set(0);
        delay_us(5);
    }
}

/*
 ** PUBLIC FUNCTION: BSP_I2C_Read()
 *
 *  DESCRIPTION:
 *      Reads the length value of the specified register for the specified device.
 *
 *  PARAMETERS:
 *      dev: Device address, reg: slave memmory address, 
 *      length: the request data length, *data: pointer to target data buffer.
 *             
 *  RETURNS:
 *      the read data.
 */
HAL_StatusTypeDef BSP_I2C_Read(uint8_t dev, uint8_t *data, uint8_t length)
{
    uint8_t count = 0;
    
    bsp_iic_start();
    bsp_iic_send_byte((dev<<1)+1);
    bsp_iic_Wait_Ack();
    delay_us(40); 
    for(count=0;count<length;count++)
    {  
        if(count!=(length-1)) 
        { 
            data[count] = bsp_iic_read_byte(1);
            delay_us(40); 
        }
        else 
        {
            data[count] = bsp_iic_read_byte(0);
        }
    }
    bsp_iic_stop();

    return count>0 ? HAL_OK : HAL_ERROR;
}

/*
 ** PUBLIC FUNCTION: bsp_iic_writeBytes()
 *
 *  DESCRIPTION:
 *      Write the length value to the specified register for the specified device.
 *
 *  PARAMETERS:
 *      dev: Device address, reg: slave memmory address, 
 *      length: the request data length, *data: pointer to data buffer.
 *             
 *  RETURNS:
 *      ture/false.
 */
HAL_StatusTypeDef BSP_I2C_Write(uint8_t dev, uint8_t* data, uint8_t length)
{
  uint8_t count = 0;

  bsp_iic_start();
  bsp_iic_send_byte(dev<<1);
  //bsp_iic_Wait_Ack();
  if (1 == bsp_iic_Wait_Ack())
  {
      delay_us(10000);                // Delay 10ms
      bsp_iic_start();                // Restart
      bsp_iic_send_byte(dev<<1);
      bsp_iic_Wait_Ack();
  }

  for(count=0;count<length;count++)
  {
      delay_us(50);
      bsp_iic_send_byte(data[count]);
      bsp_iic_Wait_Ack();
  }

  bsp_iic_stop();
  return HAL_OK;
}


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* Peripheral clock enable */
  __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/*
 ** PUBLIC FUNCTION: BSP_I2C1_Init()
 *
 *  DESCRIPTION:
 *      config the I2C pin.
 *
 *  PARAMETERS:
 *      None.
 *             
 *  RETURNS:
 *      None.
 */
void BSP_I2C1_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  __HAL_RCC_GPIOB_CLK_ENABLE();

    /**I2C GPIO Configuration
    PB8     ------> FLS110_SCL
    PB9     ------> FLS110_SDA 
    */
  GPIO_InitStruct.Pin = I2C_SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = I2C_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; //for input and output, need external pull-up
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);
  //I2C_SDA_HIGH();
  //I2C_SCL_HIGH();

  //timer2 for i2c delay
  MX_TIM2_Init();
}

#endif
