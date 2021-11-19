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
 * @file    bsp_uart.c
 * @author  QST AE team
 * @version V0.1
 * @date    2021-01-21
 * @id      $Id$
 * @brief   This file provides an implementation of the functions for the
 *          UART library.
 *
 * @note
 *
 *****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
/******************************************************
 *                      Macros
 ******************************************************/
#ifndef UNUSED_PARAMETER
#define UNUSED_PARAMETER(x) (void) x
#endif /* UNUSED_PARAMETER */

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #define GETCHAR_PROTOTYPE int __io_getchar(FILE *f)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

#define MIN(x,y)  ((x) < (y) ? (x) : (y))

/* Definition for UartStdio clock resources */
#define USART_STDIO_UART                      USART2
#define USART_STDIO_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define USART_STDIO_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART_STDIO_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

#define USART_STDIO_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USART_STDIO_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USART_STDIO Pins */
#define USART_STDIO_TX_PIN                    GPIO_PIN_2
#define USART_STDIO_TX_GPIO_PORT              GPIOA  
#define USART_STDIO_TX_AF                     GPIO_AF7_USART2
#define USART_STDIO_RX_PIN                    GPIO_PIN_3
#define USART_STDIO_RX_GPIO_PORT              GPIOA 
#define USART_STDIO_RX_AF                     GPIO_AF7_USART2

/*-----------------------------------------------------------------------------*/
/* Definition for USART_COM clock resources */
#define USART_COM_UART                        USART1
#define USART_COM_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define USART_COM_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART_COM_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

#define USART_COM_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USART_COM_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USART_COM Pins */
#define USART_COM_TX_PIN                    GPIO_PIN_9
#define USART_COM_TX_GPIO_PORT              GPIOA
#define USART_COM_TX_AF                     GPIO_AF7_USART1
#define USART_COM_RX_PIN                    GPIO_PIN_10
#define USART_COM_RX_GPIO_PORT              GPIOA
#define USART_COM_RX_AF                     GPIO_AF7_USART1

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    volatile uint32_t      rx_size;
    uint8_t*               rx_buffer;
    bool                   rx_complete;
    bool                   tx_complete;
    const uint8_t*         tx_buffer;
    volatile uint32_t      tx_size;
} uart_interface_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *                 Static Variables
 ******************************************************/
/* UART handler declaration */
static UART_HandleTypeDef UartStdio;
static UART_HandleTypeDef UartCom;
static uart_interface_t UartComInterface;

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartStdio, (uint8_t *)&ch, 1, 0xFFFF); 
  return ch;
}

GETCHAR_PROTOTYPE
{
    int ch;
    if (HAL_UART_Receive(&UartStdio, (uint8_t*)&ch, 1, 5000) != HAL_OK)
    {
        return EOF;
    }
    return ch;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  ///* Turn LED2 on */
  BSP_LED_On(LED2);
  printf("ERROR\r\n");
  while(1)
  {
  }
}

/** @brief Configure the LOG UART
  * Put the USART peripheral in the Asynchronous mode (UART Mode)
  * UART1 configured as follow:
  * - Word Length = 8 Bits
  * - Stop Bit = One Stop bit
  * - Parity = None
  * - BaudRate = 115200 baud
  * - Hardware flow control disabled (RTS and CTS signals) 
  **/
void BSP_STDIO_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  UartStdio.Instance          = USART_STDIO_UART;
  UartStdio.Init.BaudRate     = 921600;//115200;
  UartStdio.Init.WordLength   = UART_WORDLENGTH_8B;
  UartStdio.Init.StopBits     = UART_STOPBITS_1;
  UartStdio.Init.Parity       = UART_PARITY_NONE;
  UartStdio.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartStdio.Init.Mode         = UART_MODE_TX_RX;
  UartStdio.Init.OverSampling = UART_OVERSAMPLING_16;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USART_STDIO_TX_GPIO_CLK_ENABLE();
  USART_STDIO_RX_GPIO_CLK_ENABLE();

  /* Enable UartStdio clock */
  USART_STDIO_CLK_ENABLE(); 

  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USART_STDIO_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = USART_STDIO_TX_AF;
  
  HAL_GPIO_Init(USART_STDIO_TX_GPIO_PORT, &GPIO_InitStruct);
  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USART_STDIO_RX_PIN;
  GPIO_InitStruct.Alternate = USART_STDIO_RX_AF;
  HAL_GPIO_Init(USART_STDIO_RX_GPIO_PORT, &GPIO_InitStruct);

  if(HAL_UART_Init(&UartStdio) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}

/** @brief LOG UART De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param None
  * @retval None
  */
void BSP_STDIO_DeInit(void)
{
  /*##-1- Reset peripherals ##################################################*/
  USART_STDIO_FORCE_RESET();
  USART_STDIO_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(USART_STDIO_TX_GPIO_PORT, USART_STDIO_TX_PIN);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(USART_STDIO_RX_GPIO_PORT, USART_STDIO_RX_PIN);
}

//void USART_COM_IRQHandler()
//{

//}

/** @brief Configure the UART peripheral
  * Put the USART peripheral in the Asynchronous mode (UART Mode)
  * UART1 configured as follow:
  * - Word Length = 8 Bits
  * - Stop Bit = One Stop bit
  * - Parity = None
  * - BaudRate = 115200 baud
  * - Hardware flow control disabled (RTS and CTS signals) 
  **/
void BSP_COM_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  UartCom.Instance          = USART_COM_UART;
  UartCom.Init.BaudRate     = 115200;
  UartCom.Init.WordLength   = UART_WORDLENGTH_8B;
  UartCom.Init.StopBits     = UART_STOPBITS_1;
  UartCom.Init.Parity       = UART_PARITY_NONE;
  UartCom.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartCom.Init.Mode         = UART_MODE_TX_RX;
  UartCom.Init.OverSampling = UART_OVERSAMPLING_16;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USART_COM_TX_GPIO_CLK_ENABLE();
  USART_COM_RX_GPIO_CLK_ENABLE();
  /* Enable USART1 clock */
  USART_COM_CLK_ENABLE(); 

  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USART_COM_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = USART_COM_TX_AF;

  HAL_GPIO_Init(USART_COM_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USART_COM_RX_PIN;
  GPIO_InitStruct.Alternate = USART_COM_RX_AF;
    
  HAL_GPIO_Init(USART_COM_RX_GPIO_PORT, &GPIO_InitStruct);

  if(HAL_UART_Init(&UartCom) != HAL_OK)
  {
    Error_Handler();
  }
}

/** @brief COM UART De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param none
  * @retval None
  */
void BSP_COM_DeInit(void)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;

  /*##-1- Reset peripherals ##################################################*/
  USART_COM_FORCE_RESET();
  USART_COM_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(USART_COM_TX_GPIO_PORT, USART_COM_TX_PIN);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(USART_COM_RX_GPIO_PORT, USART_COM_RX_PIN);
}

int BSP_COM_Write(const uint8_t* data_out, uint32_t size)
{
  HAL_UART_Transmit(&UartCom, (uint8_t*)data_out, size, 0xF);

  return 0;
}

int BSP_COM_Read(uint8_t* data_in, uint16_t expected_data_size, uint32_t timeout_ms)
{
  HAL_StatusTypeDef rx_status;

  rx_status = HAL_UART_Receive(&UartCom, data_in, expected_data_size, timeout_ms);

  return (int)rx_status;
}

void BSP_UART_Init(void)
{
  BSP_STDIO_Init();
  BSP_COM_Init();
}

