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
 *! @file gmp102.h
 *  @brief  GMP102 Sensor Driver Header File 
 *  @author Joseph FC Tseng
 *
 * @note
 *
 *****************************************************************************/

#ifndef __GMP102_H__
#define __GMP102_H__

#ifdef __cplusplus
extern "C"
{
#endif


#define GMP102_7BIT_I2C_ADDR		0x6C
#define GMP102_8BIT_I2C_ADDR		(0x6C<<1)

#define GMP102_TEMPERATURE_SENSITIVITY 256  //1 Celsius = 256 code
#define GMP102_T_CODE_TO_CELSIUS(tCode) (((float)(tCode)) / GMP102_TEMPERATURE_SENSITIVITY)

//Registers Address
#define GMP102_REG_RESET        0x00
#define GMP102_REG_PID          0x01
#define GMP102_REG_STATUS       0x02
#define GMP102_REG_PRESSH	  0x06
#define GMP102_REG_PRESSM	  0x07
#define GMP102_REG_PRESSL	  0x08
#define GMP102_REG_TEMPH	  0x09
#define GMP102_REG_TEMPL	  0x0A
#define GMP102_REG_CMD	 	  0x30
#define GMP102_REG_CONFIG1 	  0xA5
#define GMP102_REG_CONFIG2 	  0xA6
#define GMP102_REG_CONFIG3 	  0xA7
#define GMP102_REG_CALIB00        0xAA
//Total calibration register count: AAh~BBh total 18
#define GMP102_CALIBRATION_REGISTER_COUNT 18
//Total calibration parameter count: total 9
#define GMP102_CALIBRATION_PARAMETER_COUNT (GMP102_CALIBRATION_REGISTER_COUNT/2)
//Soft reset 
#define GMP102_SW_RST_SET_VALUE		0x24


/* PID */
#define GMP102_PID__REG GMP102_REG_PID
/* Soft Rest bit */
#define GMP102_RST__REG		GMP102_REG_RESET
#define GMP102_RST__MSK		0x24
#define GMP102_RST__POS		0
/* DRDY bit */
#define GMP102_DRDY__REG	GMP102_REG_STATUS
#define GMP102_DRDY__MSK	0x01
#define GMP102_DRDY__POS	0
/* P OSR bits */
#define GMP102_P_OSR__REG       GMP102_REG_CONFIG2
#define GMP102_P_OSR__MSK       0x07
#define GMP102_P_OSR__POS       0
/* T OSR bits */
#define GMP102_T_OSR__REG       GMP102_REG_CONFIG3
#define GMP102_T_OSR__MSK       0x07
#define GMP102_T_OSR__POS       0

#define GMP102_GET_BITSLICE(regvar, bitname)	\
  ((regvar & bitname##__MSK) >> bitname##__POS)

#define GMP102_SET_BITSLICE(regvar, bitname, val)			\
  ((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

typedef enum {
  GMP102_P_OSR_256 = 0x04,
  GMP102_P_OSR_512 = 0x05,
  GMP102_P_OSR_1024 = 0x00,
  GMP102_P_OSR_2048 = 0x01,
  GMP102_P_OSR_4096 = 0x02,
  GMP102_P_OSR_8192 = 0x03,
  GMP102_P_OSR_16384 = 0x06,
  GMP102_P_OSR_32768 = 0x07,	
} GMP102_P_OSR_Type;

typedef enum {
  GMP102_T_OSR_256 = 0x04,
  GMP102_T_OSR_512 = 0x05,
  GMP102_T_OSR_1024 = 0x00,
  GMP102_T_OSR_2048 = 0x01,
  GMP102_T_OSR_4096 = 0x02,
  GMP102_T_OSR_8192 = 0x03,
  GMP102_T_OSR_16384 = 0x06,
  GMP102_T_OSR_32768 = 0x07,	
} GMP102_T_OSR_Type;



int8_t  delay1ms(int8_t cnt) ;

/*!
 * @brief Read multiple data from the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values read
 * @param u8Len Number of bytes to read
 * 
 * @return Result from the burst read function
 * @retval >= 0 Success, number of bytes read
 * @retval -127 Error null bus
 * @retval -1   Bus communication error
 *
 */
HAL_StatusTypeDef gmp102_burst_read(uint8_t u8Addr, uint8_t* pu8Data, uint8_t u8Len);

/*!
 * @brief Write multiple data to the starting regsiter address
 *
 * @param u8Addr Starting register address
 * @param pu8Data The data array of values to write
 * @param u8Len Number of bytes to write
 * 
 * @return Result from the burst write function
 * @retval >= 0 Success, number of bytes write
 * @retval -127 Error null bus
 * @retval -1   Communication error
 *
 */
HAL_StatusTypeDef gmp102_burst_write(uint8_t u8Addr, uint8_t* pu8Data, uint8_t u8Len);


/*!
 * @brief gmp102 initialize communication bus
 *
 * @param pbus Pointer to the I2C/SPI read/write bus support struct
 * 
 * @return Result from bus communication function
 * @retval 0 Success
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int8_t gmp102_get_pid(void);

/*!
 * @brief gmp102 soft reset
 *
 * @param None
 * 
 * @return Result from bus communication function
 * @retval -1 Bus communication error
 * @retval -127 Error null bus
 *
 */
int8_t gmp102_soft_reset(void);


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
int8_t gmp102_get_calibration_param(float* fCalibParam);

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
int8_t gmp102_get_calibration_param_fixed_point(int16_t s16Value[], uint8_t u8Power[]);

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
int8_t gmp102_initialization(void);


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
int8_t gmp102_measure_T(int16_t* ps16T);

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
int8_t gmp102_measure_P(int32_t* ps32P);

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
int8_t gmp102_measure_P_T(int32_t* ps32P, int16_t* ps16T, int8_t s8PWaitDrdy);

/*!
 * @brief gmp102 temperature and pressure compensation
 *
 * @param s16T calibrated temperature in code
 * @param s32P raw pressure in code
 * @param fParam[] pressure calibration parameters
 * @param *pfT_Celsius calibrated temperature in Celsius returned to caller
 * @param *pfP_Pa calibrated pressure in Pa returned to caller
 * 
 * @return None
 *
 */
void gmp102_compensation(int16_t s16T, int32_t s32P, float fParam[], float* pfT_Celsius, float* pfP_Pa);

/*!
 * @brief gmp102 temperature and pressure compensation, s64 fixed point operation
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
void gmp102_compensation_fixed_point_s64(int16_t s16T, int32_t s32P, int16_t s16Value[], uint8_t u8Power[], int32_t* ps32T_Celsius, int32_t* ps32P_Pa);

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
int8_t gmp102_set_P_OSR(GMP102_P_OSR_Type osrP);

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
int8_t gmp102_set_T_OSR(GMP102_T_OSR_Type osrT);

void MX_I2C1_Init(void);
void GMP102_Sensor_Test(void);
void gmp102_init(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // __GMP102_H__
