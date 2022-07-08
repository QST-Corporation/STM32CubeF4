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
 * @file    sw_iic.c
 * @author  QST AE team
 * @version V0.1
 * @date    2021-05-25
 * @id      $Id$
 * @brief   This file provides the functions for software I2C driver.
 *          |----------------------|
 *          |--F401RE---|--FLS110--|
 *          |----------------------|
 *          |---PB8-----|---SCL----|
 *          |---PB9-----|---SDA----|
 *          |---+3.3V---|---VDD----|
 * 
 * @note
 *
 *****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

/******************************************************
 *                      Macros
 ******************************************************/
//#define __weak __attribute__((weak))
#define I2C_SLAVE_ADDR                0x31
#define I2C_WRITE_MASK                0
#define I2C_READ_MASK                 1
#define I2C_SCL_PIN                   GPIO_PIN_8
#define I2C_SDA_PIN                   GPIO_PIN_9
#define I2C_SCL_PIN_NUM               8
#define I2C_SDA_PIN_NUM               9
#define I2C_GPIO_PORT                 GPIOB
#define I2C_SCL_HIGH()                HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET)
#define I2C_SCL_LOW()                 HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET)
#define I2C_SDA_HIGH()                HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_SET)
#define I2C_SDA_LOW()                 HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_RESET)
#define I2C_SDA_READ()                HAL_GPIO_ReadPin(I2C_GPIO_PORT, I2C_SDA_PIN)
//#define I2C_SDA_OUTPUT()              I2C_PORT_HANDL->MODER = GPIO_MODE_OUTPUT_OD << (2*I2C_SDA_PIN_NUM)
//#define I2C_SDA_INPUT()               I2C_PORT_HANDL->MODER = GPIO_MODE_INPUT << (2*I2C_SDA_PIN_NUM)

extern void Error_Handler(void);

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *                 Static Variables
 ******************************************************/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
//GPIO_TypeDef *I2C_PORT_HANDL = I2C_GPIO_PORT;

/* Private function prototypes -----------------------------------------------*/
#if 1
static void sw_i2c_delay_us(uint32_t us)
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
#else
static void sw_i2c_delay_us(uint32_t us)
{
  //uint8_t i = 5;
  while(us--);
}
#endif

static void sw_i2c_sda_dir(uint32_t mode)
{
#if 0
  GPIO_TypeDef *I2C_PORT_HANDL;
  uint32_t temp = 0;

  I2C_PORT_HANDL = I2C_GPIO_PORT;
/* Configure IO Direction mode (Input, Output, Alternate or Analog) */
  temp = I2C_PORT_HANDL->MODER;
  temp &= ~(GPIO_MODER_MODER0 << (I2C_SDA_PIN_NUM * 2U));
  temp |= ((mode & GPIO_MODE) << (I2C_SDA_PIN_NUM * 2U));
  I2C_PORT_HANDL->MODER = temp;
#else
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
#endif
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

/* I2C init function */
void sw_i2c_init(void)
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

static void sw_i2c_start(void)
{
  sw_i2c_sda_dir(GPIO_MODE_OUTPUT_OD);
  I2C_SDA_HIGH();
  //sw_i2c_delay_us(5);
  I2C_SCL_HIGH();
  sw_i2c_delay_us(10);
  I2C_SDA_LOW();
  sw_i2c_delay_us(10);
  //I2C_SCL_LOW();
  //sw_i2c_delay_us(10);
}

static void sw_i2c_stop(void)
{
  sw_i2c_sda_dir(GPIO_MODE_OUTPUT_OD);
  I2C_SDA_LOW();
  I2C_SCL_HIGH();
  sw_i2c_delay_us(10);
  I2C_SDA_LOW();
  sw_i2c_delay_us(10);
  I2C_SCL_HIGH();
  sw_i2c_delay_us(10);
  I2C_SDA_HIGH();
  sw_i2c_delay_us(10);
}

static void i2c_send_ack(void)
{
  sw_i2c_sda_dir(GPIO_MODE_OUTPUT_OD);
  //I2C_SCL_LOW();
  //sw_i2c_delay_us(5);
  I2C_SDA_LOW();
  sw_i2c_delay_us(4);
  I2C_SCL_HIGH();
  sw_i2c_delay_us(4);
  I2C_SCL_LOW();
  //sw_i2c_delay_us(4);
}

static void i2c_send_noack(void)
{
  sw_i2c_sda_dir(GPIO_MODE_OUTPUT_OD);
  //I2C_SCL_LOW();
  //sw_i2c_delay_us(5);
  I2C_SDA_HIGH();
  sw_i2c_delay_us(4);
  I2C_SCL_HIGH();
  sw_i2c_delay_us(4);
  I2C_SCL_LOW();
  //sw_i2c_delay_us(4);
}

static uint8_t sw_i2c_wait_ack(void)
{
  //I2C_SDA_HIGH();
  sw_i2c_sda_dir(GPIO_MODE_INPUT);

  sw_i2c_delay_us(4);
  I2C_SCL_HIGH();
  //sw_i2c_delay_us(4);
  if(I2C_SDA_READ())
  {
    printf("i2c wait ACK failed\n");
    //sw_i2c_stop();
    I2C_SCL_LOW();
    return HAL_ERROR;
  }
  sw_i2c_delay_us(4);
  I2C_SCL_LOW();
  //sw_i2c_delay_us(2);
  return HAL_OK; 
}

static void sw_i2c_send_byte(uint8_t data)
{
  uint8_t i = 8;

  sw_i2c_sda_dir(GPIO_MODE_OUTPUT_OD);
  while(i--)
  {
    I2C_SCL_LOW();
    sw_i2c_delay_us(2);
    if(data & 0x80)
    {
      I2C_SDA_HIGH();
    }
    else
    {
      I2C_SDA_LOW();
    }

    sw_i2c_delay_us(2);
    data <<= 1;
    I2C_SCL_HIGH();
    sw_i2c_delay_us(4);
  }
  I2C_SCL_LOW();
  //I2C_SDA_HIGH();
}

static uint8_t i2c_receive_byte(void)
{
  uint8_t i = 8;
  uint8_t data = 0;

  //I2C_SDA_HIGH();
  sw_i2c_sda_dir(GPIO_MODE_INPUT);
  //sw_i2c_delay_us(4);
  while(i--)
  {
    data <<= 1;
    I2C_SCL_LOW();
    sw_i2c_delay_us(4);
    I2C_SCL_HIGH();
    sw_i2c_delay_us(4);
    if(I2C_SDA_READ())
    {
      data |= 0x01;
    }
  }
  I2C_SCL_LOW();
  //I2C_SDA_HIGH();
  return(data);
}

HAL_StatusTypeDef sw_i2c_receive(uint8_t slave_addr, uint8_t reg_addr,
                                 uint8_t *pdata, uint8_t size)
{
  sw_i2c_start();
  sw_i2c_send_byte((slave_addr<<1) | I2C_WRITE_MASK);
  if(sw_i2c_wait_ack() != HAL_OK) {
    sw_i2c_stop();
    return HAL_ERROR;
  }
  sw_i2c_send_byte(reg_addr);
  if(sw_i2c_wait_ack() != HAL_OK) {
    sw_i2c_stop();
    return HAL_ERROR;
  }
  //sw_i2c_stop(); //????
  sw_i2c_delay_us(4);
  sw_i2c_start();
  sw_i2c_send_byte((slave_addr<<1) | I2C_READ_MASK);
  if(sw_i2c_wait_ack() != HAL_OK) {
    sw_i2c_stop();
    return HAL_ERROR;
  }
  while(size)
  {
    *pdata = i2c_receive_byte();
    if (size == 1) {
      i2c_send_noack();
    } else {
      i2c_send_ack();
    }
    pdata++;
    size--;
  }

  sw_i2c_stop();
  return HAL_OK;
}

HAL_StatusTypeDef sw_i2c_transmit(uint8_t slave_addr, uint8_t reg_addr,
                                   uint8_t *pdata, uint8_t size)
{
  sw_i2c_start();

  sw_i2c_send_byte((slave_addr<<1) | I2C_WRITE_MASK);
  if(sw_i2c_wait_ack() != HAL_OK) {
    sw_i2c_stop();
    return HAL_ERROR;
  }

  sw_i2c_send_byte(reg_addr);
  if(sw_i2c_wait_ack() != HAL_OK) {
    sw_i2c_stop();
    return HAL_ERROR;
  }

  while(size--)
  {
    sw_i2c_send_byte(*pdata);
    if(sw_i2c_wait_ack() != HAL_OK) {
      sw_i2c_stop();
      return HAL_ERROR;
    }
    pdata++;
  }

  sw_i2c_stop();
  return HAL_OK;
}

void sw_i2c_scl_toggle(void)
{
  I2C_SCL_LOW();
  sw_i2c_delay_us(5);
  I2C_SCL_HIGH();
  sw_i2c_delay_us(5);
}

void sw_i2c_sda_toggle(void)
{
  sw_i2c_sda_dir(GPIO_MODE_OUTPUT_OD);
  I2C_SDA_LOW();
  sw_i2c_delay_us(5);
  I2C_SDA_HIGH();
  sw_i2c_delay_us(5);
}

