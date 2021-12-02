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
 * @file    blue_air_speed/Src/main.c
 * @author  QST AE team
 * @version V0.1
 * @date    2021-05-28
 * @id      $Id$
 * @brief   This code used to test QST QME7E00 and TE MS4525 sensor and output the data via debug UART.
 *
 * @note
 *
 *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "main.h"
#include "bsp_uart.h"
#include "version_print.h"
#include "air_speed_i2c.h"
#include "qme7e00.h"
#include "ms4525do.h"
#include "SPL06_01.h"
#include "flash.h"
#include "math.h" //for sqrtf()

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define AS_SENSOR_DATA_STORE_ENABLE       false//true
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef    AirSpeedTimHandle;
uint32_t splLastUpdateTime;
uint8_t flashData[1040];
volatile bool sensorIsRunning = false;
volatile bool sensorTimerPeriodElapsed = false;
extern volatile bool sensorEnable;
extern bool uartFlashReadCmd;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  ///* Turn LED2 on */
  BSP_LED_On(LED2);
  printf("ERROR\r\n");
  while(1)
  {
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void BSP_Device_Init(void)
{
  BSP_Button_Init();

  /* Configure LED2 */
  BSP_LED_Init(LED2);

  /* Configure LOG UART and Communication UART */
  BSP_UART_Init();
}

static void AirSpeedSensor_Timer_Init_And_Start(uint32_t periodMs)
{
 /*##-1- Configure the TIM peripheral #######################################*/ 
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1
      PCLK1 = HCLK / 2
      => TIM3CLK = HCLK = SystemCoreClock
    To get TIM3 counter clock at 1KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = (SystemCoreClock /1000) - 1
       
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  

  /*##-1.1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  __HAL_RCC_TIM3_CLK_ENABLE();
  /*##-1.2- Configure the NVIC for TIMx ########################################*/
  /* Set Interrupt Group Priority */ 
  HAL_NVIC_SetPriority(TIM3_IRQn, 4, 0);
  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  /* Compute the prescaler value to have TIM3 counter clock equal to periodMs */
  uint32_t uwPrescalerValue = (uint32_t) (((SystemCoreClock/2) / 1000) - 1);

  /* Set TIM3 instance */
  AirSpeedTimHandle.Instance = TIM3;
   
  /* Initialize TIM3 peripheral as follow:
       + Period = periodMs - 1
       + Prescaler = ((SystemCoreClock/2)/periodMs) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  AirSpeedTimHandle.Init.Period = periodMs*2 - 1;
  AirSpeedTimHandle.Init.Prescaler = uwPrescalerValue;
  AirSpeedTimHandle.Init.ClockDivision = 0;
  AirSpeedTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  AirSpeedTimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_Base_Init(&AirSpeedTimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&AirSpeedTimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}

static void AirSpeedSensor_Timer_Stop(void)
{
  if(HAL_TIM_Base_Stop_IT(&AirSpeedTimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}

// 输入：压差、静压、当地温度（华氏摄氏度）
static float cal_true_airspeed(float diff_pressure, float pressure_static, float tempreture_cel){
    //float AIR_DENSITY_SEA_LEVEL_15CEL = 1.225f;
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
static float cal_true_airspeed_lite(float diff_pressure){
    float AIR_DENSITY_SEA_LEVEL_15CEL = 1.225;
    float true_airspeed = sqrtf(2.0f* diff_pressure / AIR_DENSITY_SEA_LEVEL_15CEL);
    return true_airspeed;
}

static uint16_t sampleRate = 0;
void AirSpeedSensorsFetchData(void)
{
  uint8_t i, flashRet=0;
  float qmeSpeed = 0.0f, qmeDP = 0.0f, qmeTemp = 0.0f;
  float msSpeed = 0.0f, msPress = 0.0f, msTemp = 0.0f;
  float splPress = 0.0f;
  uint16_t msRaw = 0;
  uint32_t qmeRaw = 0;
  uint32_t msPressRawAndTemp = 0;
  uint32_t qmeTimestamp = 0, msTimestamp = 0, splTimestamp = 0, freshTimestamp;
  uint32_t sensorsSample[4] = {0,};
  static uint8_t fistRunFlag = 0, sensorDataLen = 0;
  static float pressure_static = 101325.0f;
  static float tempreture_cel = 78.8f;

  if (fistRunFlag == 0) {
    fistRunFlag = 1;
    splLastUpdateTime = HAL_GetTick();
  }
  QME7E00_Sensor_Update(&qmeRaw, &qmeDP, &qmeTemp, &qmeTimestamp);
  freshTimestamp = HAL_GetTick();
  qmeSpeed = cal_true_airspeed(qmeDP, pressure_static, tempreture_cel);
  HAL_Delay(5);
  MS4525DO_Sensor_Update(&msRaw, &msPress, &msTemp, &msTimestamp);
  msPressRawAndTemp = ((uint32_t)msRaw << 16 ) | (uint32_t)(msTemp*10);
  msSpeed = cal_true_airspeed(msPress, pressure_static, tempreture_cel);
  sensorsSample[0] = freshTimestamp;
  sensorsSample[1] = qmeRaw;//(uint32_t)(qmeDP*100);
  sensorsSample[2] = msPressRawAndTemp;//(msPress*100);
  if ((freshTimestamp - splLastUpdateTime) > 1000) {
    spl0601_update_pressure(&splPress, &splTimestamp);
    splLastUpdateTime = freshTimestamp;
    sensorsSample[3] = (uint32_t)(splPress*100);
    sensorDataLen = 4;
    //printf("%ld: 16, %d\n", splLastUpdateTime, sampleRate+1);
    if(splPress > 0) {
      pressure_static = splPress; //update the static pressure.
    }
    sampleRate = 0;
    //timestamp: MS_dp, QME_dp, MS_TAS, QME_TAS, MS_temp, QME_temp, barometer
    printf("%02d:%02d:%03d %.2f, %.2f, %.2f, %.2f, %.1f, %.1f, %.2f\n", \
            freshTimestamp/60000, (freshTimestamp%60000)/1000, (freshTimestamp%60000)%1000, \
            msPress, qmeDP, msSpeed, qmeSpeed, msTemp, qmeTemp, splPress);
  } else {
    sensorDataLen = 3;
    sampleRate ++;
    //timestamp: MS_dp, QME_dp, MS_TAS, QME_TAS, MS_temp, QME_temp
    printf("%02d:%02d:%03d %.2f, %.2f, %.2f, %.2f, %.1f, %.1f\n", \
            freshTimestamp/60000, (freshTimestamp%60000)/1000, (freshTimestamp%60000)%1000, \
            msPress, qmeDP, msSpeed, qmeSpeed, msTemp, qmeTemp);
  }

#if (AS_SENSOR_DATA_STORE_ENABLE)
  //store sensor data to flash:
  for (i=0; i<sensorDataLen; i++) {
    flashRet = Store_Data((uint8_t *)&sensorsSample[i], sizeof(uint32_t));
    if (flashRet == 0) {
      sensorEnable = false;
      printf("Store_Data: 0\n");
      break;
    }
  }
#else //#if (AS_SENSOR_DATA_STORE_ENABLE)
  UNUSED(i);
  UNUSED(flashRet);
  UNUSED(sensorDataLen);
  UNUSED(sensorsSample);
#endif //#if (AS_SENSOR_DATA_STORE_ENABLE)

}

void UartFlashReadHanlder(void)
{
#if (AS_SENSOR_DATA_STORE_ENABLE)
  uint8_t dataAvailable = 0;
  uint32_t sensorSamples[4] = {0};
  uint32_t splLastUpdateTime = 0, byteCnt = 0, flashByteCnt = 0;
  uint16_t index = 0;
  uint8_t restData[16] = {0,}; //1040-1024
  uint8_t restDataCnt = 0;
  uint16_t msRaw;
  float msTemp = 0.0f;

  //printf("uartFlashReadCmd\n");
  memset(flashData, 0x00, sizeof(flashData));
  dataAvailable = Read_Data(flashData);
  flashByteCnt = 1024;
  memmove(&splLastUpdateTime, flashData, 4);
  while (dataAvailable){
    for (index=0; byteCnt+16<flashByteCnt; index++) {
      memmove((uint8_t *)sensorSamples, &flashData[byteCnt], 12);
      byteCnt += 12;
      msRaw = sensorSamples[2] >> 16;
      msTemp = (float)(sensorSamples[2] & 0xFFFF)/10;
      if(sensorSamples[0] - splLastUpdateTime > 1000) {
        splLastUpdateTime = sensorSamples[0];
        memmove((uint8_t *)&sensorSamples[3], &flashData[byteCnt], 4);
        byteCnt += 4;
        printf("%ld, %ld, %d, %.1f, %.2f\n", sensorSamples[0], sensorSamples[1], msRaw, msTemp/*+0.1f*/, ((float)sensorSamples[3]/100));
      } else {
        printf("%ld, %ld, %d, %.1f\n", sensorSamples[0], sensorSamples[1], msRaw, msTemp/*+0.1f*/);
      }
    }
    if (byteCnt < flashByteCnt) {
      restDataCnt = flashByteCnt - byteCnt;
      if (restDataCnt < sizeof(restData)+1) {
        memmove(restData, &flashData[byteCnt], restDataCnt);
      } else {
        printf("Flash data parse error!\n");
        return;
      }
    }
    else {
      restDataCnt = 0;
    }
    //printf("flashByteCnt %d, byteCnt %d, rest %d\n", flashByteCnt, byteCnt, restDataCnt);
    HAL_Delay(100);
    memset(flashData, 0x00, sizeof(flashData));
    memmove(flashData, restData, restDataCnt);
    index = 0;
    byteCnt = 0;
    dataAvailable = Read_Data(&flashData[restDataCnt]);
    flashByteCnt = restDataCnt + 1024;
  }
#else  //#if (AS_SENSOR_DATA_STORE_ENABLE)
  printf("flash store is disabled\n");
#endif //#if (AS_SENSOR_DATA_STORE_ENABLE)
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //it's in ISR, should not invoke the sensor data sampling here.
  //instead of issuing event.
  sensorTimerPeriodElapsed = true;
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();

  /* Configure the system clock to 84 MHz */
  SystemClock_Config();

  BSP_Device_Init();

  /* Log version info */
  PrintVersion();

  /* Perform I2C initialization for Air Speed sensors*/
  Air_Speed_I2C1_Init();
  QME7E00_Sensor_Init();
  spl0601_init_and_start();
  Check_data();
  //splLastUpdateTime = HAL_GetTick();

  /* Infinite loop */
  while (1)
  {
    if (sensorEnable) {
      if(!sensorIsRunning)
      {
        sensorIsRunning = true;
        AirSpeedSensor_Timer_Init_And_Start(1000);
      }
      if(sensorTimerPeriodElapsed)
      {
        sensorTimerPeriodElapsed = false;
        AirSpeedSensorsFetchData();
      }
    } else {
      if(sensorIsRunning)
      {
        sensorIsRunning = false;
        AirSpeedSensor_Timer_Stop();
      }
      if (uartFlashReadCmd) {
        UartFlashReadHanlder();
        uartFlashReadCmd = false;
      }
    }
    BSP_Button_Polling();
  }
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT QST Corporation *****END OF FILE****/
