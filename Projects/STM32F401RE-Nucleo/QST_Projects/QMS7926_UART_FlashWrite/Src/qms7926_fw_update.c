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
 * @file    qms7926_fw_update.c
 * @author  QST AE team
 * @version V0.1
 * @date    2021-01-22
 * @id      $Id$
 * @brief   This file provides the functions for programming QMS7926 via UART.
 *
 * @note
 *
 *****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "bsp_uart.h"
#include "image\image.h"

/******************************************************
 *                      Macros
 ******************************************************/
//#define __weak __attribute__((weak))
#define QMS7926_UART_TIMEOUT_MS           500
#define QMS7926_CMD_CHIPERASE             "er512"
#define QMS7926_CMD_CPNUM2                "cpnum 2"
#define QMS7926_RSP_CMD_OK                "#OK>>:"
#define QMS7926_RSP_CPBIN                 "by hex mode: "

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *                 Static Variables
 ******************************************************/

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  uint16_t i;
  for (i=1; i<BufferLength+1; i++) {
    if ((*pBuffer1) != (*pBuffer2))
    {
      return i;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return SUCCESS;
}

static int qms7926_boot_entry(void)
{
#if 0
  uint8_t rxStr[10] = {0x00,};
  const char expected_prompt[] = "cmd>>:";

  /* 1. drive TM pin HIGH */

  /* 2, hardware reset */

  /* 3. check prompt */
  BSP_COM_Read(rxStr, strlen(expected_prompt), QMS7926_UART_TIMEOUT_MS);
  printf("prompt: %s", rxStr);

  if (buffercmp(rxStr, (uint8_t *)expected_prompt, strlen(expected_prompt)) != 0) {
    return -1;
  }
#endif
  return SUCCESS;
}

static int qms7926_boot_exit(void)
{
  /* 1. drive TM pin LOW */

  /* 2, hardware reset */

  return SUCCESS;
}

static int qms7926_uart_flash_cmd(const char *pcmd, uint16_t cmdLen, const char *expected_rsp, uint8_t expected_rsp_len)
{
  uint8_t rxStr[10] = {0x00,};

  BSP_COM_Write((const uint8_t*)pcmd, cmdLen);
  BSP_COM_Read(rxStr, expected_rsp_len, QMS7926_UART_TIMEOUT_MS);
  printf("Rsp:[%s],expected:[%s],expected_len:%d\n", rxStr, expected_rsp, expected_rsp_len);

  if (buffercmp(rxStr, (uint8_t *)expected_rsp, expected_rsp_len) != 0) {
    return -1;
  }
  return SUCCESS;
}

static int qms7926_write_flash(const uint8_t bin_file_num)
{
  uint8_t rxStr[50] = {0x00,};
  uint8_t fragment_len = 0, i;
  const char *p_cpbin_cmd;
  const char *p_chksum;
  const uint8_t *p_bin;
  uint16_t cpbin_cmd_len = 0, chksum_len = 0;
  uint32_t bin_size = 0;
  const char expected_rsp[] = "checksum is: 0xXXXXXXXX ";
  int ret;

  for (i=0; i<bin_file_num; i++) {
    if (i == 0) {
      p_cpbin_cmd = bin1_cpbin_cmd;
      cpbin_cmd_len = strlen(bin1_cpbin_cmd);
      p_chksum = bin1_chksum;
      chksum_len = strlen(bin1_chksum);
      p_bin = bin1;
      bin_size = bin1_char_len;
    }
    else if (i == 1) {
      p_cpbin_cmd = bin2_cpbin_cmd;
      cpbin_cmd_len = strlen(bin2_cpbin_cmd);
      p_chksum = bin2_chksum;
      chksum_len = strlen(bin2_chksum);
      p_bin = bin2;
      bin_size = bin2_char_len;
    }
    else if (i == 2) {
      p_cpbin_cmd = bin3_cpbin_cmd;
      cpbin_cmd_len = strlen(bin3_cpbin_cmd);
      p_chksum = bin3_chksum;
      chksum_len = strlen(bin3_chksum);
      p_bin = bin3;
      bin_size = bin3_char_len;
    }
    else {
      return -1;
    }

    //ret = qms7926_uart_flash_cmd(QMS7926_CMD_CPNUM2, strlen(QMS7926_CMD_CPNUM2),
    //                             QMS7926_RSP_CMD_OK, strlen(QMS7926_RSP_CMD_OK));
    //printf("cpunum 2: %d\n", ret);
    //if (ret != SUCCESS) {
    //  return ret;
    //}

    ret = qms7926_uart_flash_cmd(p_cpbin_cmd, cpbin_cmd_len,
                                 QMS7926_RSP_CPBIN, strlen(QMS7926_RSP_CPBIN));
    printf("Tx[%s]: %d\n", p_cpbin_cmd, ret);
    if (ret != SUCCESS) {
      return ret;
    }

    while (bin_size) {
      fragment_len = bin_size > 40 ? 40 : bin_size;
      BSP_COM_Write(p_bin, fragment_len);
      p_bin += fragment_len;
      bin_size -= fragment_len;
    }
    ret = BSP_COM_Read(rxStr, strlen(expected_rsp), QMS7926_UART_TIMEOUT_MS*2);
    printf("Rx[%s]: %d\n", rxStr, ret);
    if (ret != SUCCESS) {
      return ret;
    }

    ret = qms7926_uart_flash_cmd(p_chksum, chksum_len,
                                 QMS7926_RSP_CMD_OK, strlen(QMS7926_RSP_CMD_OK));
    printf("Tx[%s]: %d\n", p_chksum, ret);
    if (ret != SUCCESS) {
      return ret;
    }
  }

  return SUCCESS;
}

int qms7926_fw_update(void)
{
  int ret = 0;

  /* 1, Enter into boot mode */
  ret = qms7926_boot_entry();
  if (ret != SUCCESS) {
    return ret;
  }

  /* 2, Erase entire chip */
  ret = qms7926_uart_flash_cmd(QMS7926_CMD_CHIPERASE, strlen(QMS7926_CMD_CHIPERASE),
                               QMS7926_RSP_CMD_OK, strlen(QMS7926_RSP_CMD_OK));
  printf("erase:  %s\n", ret == SUCCESS ? "successfully" : "failed");
  if (ret != SUCCESS) {
    return ret;
  }

  /* 3, Download image data */
  ret = qms7926_write_flash(bin_total_num);
  printf("Image download %s\n", ret == SUCCESS ? "successfully" : "failed");
  if (ret != SUCCESS) {
    return ret;
  }

  /* 4, Exit boot mode to run new iamge */
  ret = qms7926_boot_exit();

  return ret;
}
