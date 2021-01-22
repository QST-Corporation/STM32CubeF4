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
 * @file    qms7926_flashwrite_uart.c
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
#define USART_COM_DMA_CLK_ENABLE()          __HAL_RCC_DMA1_CLK_ENABLE()
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

/* Definition for USART_COM's DMA */
#define USART_COM_TX_DMA_CHANNEL            DMA_CHANNEL_4
#define USART_COM_TX_DMA_STREAM             DMA1_Stream6
#define USART_COM_RX_DMA_CHANNEL            DMA_CHANNEL_4
#define USART_COM_RX_DMA_STREAM             DMA1_Stream5

/* Definition for USART_COM's NVIC */
#define USART_COM_DMA_TX_IRQn               DMA1_Stream6_IRQn
#define USART_COM_DMA_RX_IRQn               DMA1_Stream5_IRQn
#define USART_COM_DMA_TX_IRQHandler         DMA1_Stream6_IRQHandler
#define USART_COM_DMA_RX_IRQHandler         DMA1_Stream5_IRQHandler
#define USART_COM_IRQn                      USART1_IRQn
#define USART_COM_IRQHandler                USART1_IRQHandler

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
  UartStdio.Init.BaudRate     = 115200;
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
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;
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
  /* Enable DMA1 clock */
  USART_COM_DMA_CLK_ENABLE();   

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
    
  /*##-3- Configure the DMA streams ##########################################*/
  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = USART_COM_TX_DMA_STREAM;
  hdma_tx.Init.Channel             = USART_COM_TX_DMA_CHANNEL;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

  HAL_DMA_Init(&hdma_tx);   

  /* Associate the initialized DMA handle to the the UART handle */
  __HAL_LINKDMA(&UartCom, hdmatx, hdma_tx);

  /* Configure the DMA handler for Transmission process */
  hdma_rx.Instance                 = USART_COM_RX_DMA_STREAM;
  hdma_rx.Init.Channel             = USART_COM_RX_DMA_CHANNEL;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode                = DMA_NORMAL;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
  hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4; 

  HAL_DMA_Init(&hdma_rx);

  /* Associate the initialized DMA handle to the the UART handle */
  __HAL_LINKDMA(&UartCom, hdmarx, hdma_rx);

  /*##-4- Configure the NVIC for DMA #########################################*/
  /* NVIC configuration for DMA transfer complete interrupt (USART_COM_TX) */
  HAL_NVIC_SetPriority(USART_COM_DMA_TX_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART_COM_DMA_TX_IRQn);

  /* NVIC configuration for DMA transfer complete interrupt (USART_COM_RX) */
  HAL_NVIC_SetPriority(USART_COM_DMA_RX_IRQn, 0, 0);   
  HAL_NVIC_EnableIRQ(USART_COM_DMA_RX_IRQn);

  /* NVIC configuration for USART TC interrupt */
  HAL_NVIC_SetPriority(USART_COM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART_COM_IRQn);

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

  /*##-3- Disable the DMA Streams ############################################*/
  /* De-Initialize the DMA Stream associate to transmission process */
  HAL_DMA_DeInit(&hdma_tx); 
  /* De-Initialize the DMA Stream associate to reception process */
  HAL_DMA_DeInit(&hdma_rx);

  /*##-4- Disable the NVIC for DMA ###########################################*/
  HAL_NVIC_DisableIRQ(USART_COM_DMA_TX_IRQn);
  HAL_NVIC_DisableIRQ(USART_COM_DMA_RX_IRQn);
}

int BSP_COM_Write(const uint8_t* data_out, uint32_t size)
{
  HAL_UART_Transmit(&UartCom, (uint8_t*)data_out, size, 0xF);

  return 0;
}

#if 0
int BSP_COM_Read(uint8_t* data_in, uint32_t expected_data_size, uint32_t timeout_ms)
{
    uint8_t* available_data2;
    uint32_t bytes_available2;

    while (expected_data_size != 0)
    {
        uint32_t transfer_size = MIN(UartComInterface.rx_buffer->size / 2, expected_data_size);

        /* Check if ring buffer already contains the required amount of data. */
        if ( transfer_size > ring_buffer_used_space( UartComInterface.rx_buffer ) )
        {
            /* Set rx_size and wait in rx_complete semaphore until data reaches rx_size or timeout occurs */
            UartComInterface.rx_size = transfer_size;

            if ( host_rtos_get_semaphore( &UartComInterface.rx_complete, timeout_ms, true ) != 0 )
            {
                UartComInterface.rx_size = 0;
#if 1
                ring_buffer_get_data( UartComInterface.rx_buffer, &available_data2, &bytes_available2 );

                while ( bytes_available2 > 0 )
                {
                    bytes_available2 = MIN( bytes_available2, transfer_size );
                    memcpy( data_in, available_data2, bytes_available2 );
                    data_in = ( (uint8_t*) data_in + bytes_available2 );
                    ring_buffer_consume( UartComInterface.rx_buffer, bytes_available2 );
                    ring_buffer_get_data( UartComInterface.rx_buffer, &available_data2, &bytes_available2 );
                }

#endif
                return -2;
            }

            /* Reset rx_size to prevent semaphore being set while nothing waits for the data */
            UartComInterface.rx_size = 0;
        }

        expected_data_size -= transfer_size;

        // Grab data from the buffer
        do
        {
            uint8_t* available_data;
            uint32_t bytes_available;

            ring_buffer_get_data( UartComInterface.rx_buffer, &available_data, &bytes_available );
            bytes_available = MIN( bytes_available, transfer_size );
            memcpy( data_in, available_data, bytes_available );
            transfer_size -= bytes_available;
            data_in = ( (uint8_t*) data_in + bytes_available );
            ring_buffer_consume( UartComInterface.rx_buffer, bytes_available );
        }
        while ( transfer_size != 0 );
    }

    if ( expected_data_size != 0 )
    {
        return -1;
    }
    else
    {
        return 0;
    }
}
#endif
void BSP_UART_Init(void)
{
  BSP_STDIO_Init();
  BSP_COM_Init();
}

#if 1
/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/
  //UartReady = SET;

  ///* Turn LED6 on: Transfer in transmission process is correct */
  //BSP_LED_On(LED6); 
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/
  //UartReady = SET;
  
  ///* Turn LED4 on: Transfer in reception process is correct */
  //BSP_LED_On(LED4);
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  /* Turn LED2 on: Transfer error in reception/transmission process */
  BSP_LED_On(LED2);
  printf("ERROR: UART transfer error\r\n");
}

#endif
