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
 * @file    version_print.c
 * @author  QST AE team
 * @version V0.1
 * @date    2021-01-22
 * @id      $Id$
 * @brief   This file provides the functions for version info log.
 *
 * @note
 *
 *****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
/******************************************************
 *                      Macros
 ******************************************************/
#define VER(R)                        #R
#define VERMACRO(R)                   VER(R)

/* Private function prototypes -----------------------------------------------*/


/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *                 Static Variables
 ******************************************************/
static char app_ver_str[50] = {"0.0.02"};
const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug",
                        "Sep", "Oct", "Nov", "Dec"};

static void getdate(char* pDest, uint8_t size)
{
  char temp[] = __DATE__;
  uint8_t i;
  uint8_t month = 0, day, year;

  if((pDest == NULL) || (size < 18)) {
    return;
  }

  year = atoi(temp + 9);
  *(temp + 6) = 0;
  day = atoi(temp + 4);
  *(temp + 3) = 0;
  for (i = 0; i < 12; i++)
  {
    if (!strcmp(temp, months[i]))
    {
      month = i + 1;
      break;
    }
  }
  *pDest++ = (year%100)/10+0x30;
  *pDest++ = year%10+0x30;
  *pDest++ = '-';
  *pDest++ = (month%100)/10+0x30;
  *pDest++ = month%10+0x30;
  *pDest++ = '-';
  *pDest++ = (day%100)/10+0x30;
  *pDest++ = day%10+0x30;
  *pDest++ = ' ';
  memcpy(pDest,__TIME__,strlen(__TIME__));
}

static char* get_app_ver(void)
{
  //sprintf(app_ver_str, "%s", VERMACRO(APP_BUILD_VER));
  return app_ver_str;
}

void PrintVersion(void)
{
  char time[20];
  memset(time, 0, sizeof(time));
  getdate(time, sizeof(time));

  printf( "\r\n" );
  printf( "=========================================================\r\n" );
  printf( "             QST Corporation Ltd.\r\n" );
  printf( "            GMP102 Sensor Test\r\n" );
  printf( "             (Version %s)\r\n", get_app_ver());
  printf( "         [Build Time: 20%s]\r\n", time);
  //printf( "               --By %s\r\n", AUTHOR);
  printf( "=========================================================\r\n" );
}
