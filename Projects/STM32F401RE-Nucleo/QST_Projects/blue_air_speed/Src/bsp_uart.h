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
  
*******************************************************************************/

/******************************************************************************
 * @file    bsp_uart.h
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

#ifndef _BSP_UART_H_
#define _BSP_UART_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define AIRSPEED_START_EVT            0x0001
#define AIRSPEED_STOP_EVT             0x0002
#define AIRSPEED_BUTTON_TOGGLE_EVT    0x0004
#define AIRSPEED_TIMER_TIMEOUT_EVT    0x0008
#define AIRSPEED_FLASH_READ_EVT       0x0010
#define AIRSPEED_FLASH_ERASE_EVT      0x0020

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *             Function Declarations
 ******************************************************/
int BSP_COM_Write(const uint8_t* data_out, uint16_t size);
int BSP_COM_Read(uint8_t* data_in, uint16_t expected_data_size, uint32_t timeout_ms);
//int BSP_COM_BytesInBuffer(void);
//void BSP_COM_IRQHandler(void);
void BSP_UART_Init(void);
void BSP_STDIO_Read(uint8_t *data, uint16_t size);
void BSP_Button_Init(void);
void BSP_Button_Polling(void);
void bsp_set_event(uint16_t evt);
uint16_t bsp_get_event(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _BSP_UART_H_ */