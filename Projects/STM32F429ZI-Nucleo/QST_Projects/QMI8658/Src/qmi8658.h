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
 * @file    qmi8658.h
 * @author  QST AE team
 * @version V0.1
 * @date    2021-09-03
 * @id      $Id$
 * @brief   This file provides the functions for QST QMI8658 sensor evaluation.
 *
 * @note
 *
 *****************************************************************************/

#ifndef _QMI8658_H_
#define _QMI8658_H_

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define QMI8658_USE_FIFO
//#define QMI8658_SYNC_SAMPLE_MODE

#ifndef M_PI
#define M_PI	(3.14159265358979323846f)
#endif
#ifndef ONE_G
#define ONE_G	(9.807f)
#endif

#define QMI8658_CTRL7_DISABLE_ALL			(0x0)
#define QMI8658_CTRL7_ACC_ENABLE			(0x1)
#define QMI8658_CTRL7_GYR_ENABLE			(0x2)
#define QMI8658_CTRL7_MAG_ENABLE			(0x4)
#define QMI8658_CTRL7_AE_ENABLE				(0x8)
#define QMI8658_CTRL7_GYR_SNOOZE_ENABLE		(0x10)
#define QMI8658_CTRL7_ENABLE_MASK			(0xF)

#define QMI8658_CONFIG_ACC_ENABLE			QMI8658_CTRL7_ACC_ENABLE
#define QMI8658_CONFIG_GYR_ENABLE			QMI8658_CTRL7_GYR_ENABLE
#define QMI8658_CONFIG_MAG_ENABLE			QMI8658_CTRL7_MAG_ENABLE
#define QMI8658_CONFIG_AE_ENABLE			QMI8658_CTRL7_AE_ENABLE
#define QMI8658_CONFIG_ACCGYR_ENABLE		(QMI8658_CONFIG_ACC_ENABLE | QMI8658_CONFIG_GYR_ENABLE)
#define QMI8658_CONFIG_ACCGYRMAG_ENABLE		(QMI8658_CONFIG_ACC_ENABLE | QMI8658_CONFIG_GYR_ENABLE | QMI8658_CONFIG_MAG_ENABLE)
#define QMI8658_CONFIG_AEMAG_ENABLE			(QMI8658_CONFIG_AE_ENABLE | QMI8658_CONFIG_MAG_ENABLE)

#define QMI8658_STATUS1_CMD_DONE			(0x01)
#define QMI8658_STATUS1_WAKEUP_EVENT		(0x04)

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/
 
/******************************************************
 *                    Structures
 ******************************************************/

enum Qmi8658Register
{
	Qmi8658Register_WhoAmI = 0,
	Qmi8658Register_Revision,
	Qmi8658Register_Ctrl1,
	Qmi8658Register_Ctrl2,
	Qmi8658Register_Ctrl3,
	Qmi8658Register_Ctrl4,
	Qmi8658Register_Ctrl5,
	Qmi8658Register_Ctrl6,
	Qmi8658Register_Ctrl7,
	Qmi8658Register_Ctrl8,
	Qmi8658Register_Ctrl9,
	Qmi8658Register_Cal1_L = 11,
	Qmi8658Register_Cal1_H,
	Qmi8658Register_Cal2_L,
	Qmi8658Register_Cal2_H,
	Qmi8658Register_Cal3_L,
	Qmi8658Register_Cal3_H,
	Qmi8658Register_Cal4_L,
	Qmi8658Register_Cal4_H,
	Qmi8658Register_FifoWmkTh = 19, 
	Qmi8658Register_FifoCtrl = 20,
	Qmi8658Register_FifoCount = 21,
	Qmi8658Register_FifoStatus = 22,
	Qmi8658Register_FifoData = 23,
	Qmi8658Register_StatusI2CM = 44,
	Qmi8658Register_StatusInt = 45,
	Qmi8658Register_Status0,
	Qmi8658Register_Status1,
	Qmi8658Register_Timestamp_L = 48,
	Qmi8658Register_Timestamp_M,
	Qmi8658Register_Timestamp_H,
	Qmi8658Register_Tempearture_L = 51,
	Qmi8658Register_Tempearture_H,
	Qmi8658Register_Ax_L = 53,
	Qmi8658Register_Ax_H,
	Qmi8658Register_Ay_L,
	Qmi8658Register_Ay_H,
	Qmi8658Register_Az_L,
	Qmi8658Register_Az_H,
	Qmi8658Register_Gx_L = 59,
	Qmi8658Register_Gx_H,
	Qmi8658Register_Gy_L,
	Qmi8658Register_Gy_H,
	Qmi8658Register_Gz_L,
	Qmi8658Register_Gz_H,
	Qmi8658Register_Mx_L = 65,
	Qmi8658Register_Mx_H,
	Qmi8658Register_My_L,
	Qmi8658Register_My_H,
	Qmi8658Register_Mz_L,
	Qmi8658Register_Mz_H,
	Qmi8658Register_Q1_L = 73,
	Qmi8658Register_Q1_H,
	Qmi8658Register_Q2_L,
	Qmi8658Register_Q2_H,
	Qmi8658Register_Q3_L,
	Qmi8658Register_Q3_H,
	Qmi8658Register_Q4_L,
	Qmi8658Register_Q4_H,
	Qmi8658Register_Dvx_L = 81,
	Qmi8658Register_Dvx_H,
	Qmi8658Register_Dvy_L,
	Qmi8658Register_Dvy_H,
	Qmi8658Register_Dvz_L,
	Qmi8658Register_Dvz_H,
	Qmi8658Register_AeReg1 = 87,
	Qmi8658Register_AeOverflow,

	Qmi8658Register_AccEl_X = 90,
	Qmi8658Register_AccEl_Y,
	Qmi8658Register_AccEl_Z,

  Qmi8658Register_SW_Reset = 96,
	Qmi8658Register_I2CM_STATUS = 110
};

enum Qmi8658_Ois_Register
{
	Qmi8658_OIS_Reg_Ctrl1 = 0x02,
	Qmi8658_OIS_Reg_Ctrl2,
	Qmi8658_OIS_Reg_Ctrl3,
	Qmi8658_OIS_Reg_Ctrl5	= 0x06,
	Qmi8658_OIS_Reg_Ctrl7   = 0x08,
	Qmi8658_OIS_Reg_StatusInt = 0x2D,
	Qmi8658_OIS_Reg_Status0   = 0x2E,
	Qmi8658_OIS_Reg_Ax_L  = 0x33,
	Qmi8658_OIS_Reg_Ax_H,
	Qmi8658_OIS_Reg_Ay_L,
	Qmi8658_OIS_Reg_Ay_H,
	Qmi8658_OIS_Reg_Az_L,
	Qmi8658_OIS_Reg_Az_H,

	Qmi8658_OIS_Reg_Gx_L  = 0x3B,
	Qmi8658_OIS_Reg_Gx_H,
	Qmi8658_OIS_Reg_Gy_L,
	Qmi8658_OIS_Reg_Gy_H,
	Qmi8658_OIS_Reg_Gz_L,
	Qmi8658_OIS_Reg_Gz_H,
};

enum Qmi8658_Ctrl9Command
{
	Qmi8658_Ctrl9_Cmd_NOP					= 0X00,
	Qmi8658_Ctrl9_Cmd_GyroBias				= 0X01,
	Qmi8658_Ctrl9_Cmd_Rqst_Sdi_Mod			= 0X03,
	Qmi8658_Ctrl9_Cmd_Rst_Fifo				= 0X04,
	Qmi8658_Ctrl9_Cmd_Req_Fifo				= 0X05,
	Qmi8658_Ctrl9_Cmd_I2CM_Write			= 0X06,
	Qmi8658_Ctrl9_Cmd_WoM_Setting			= 0x08,
	Qmi8658_Ctrl9_Cmd_AccelHostDeltaOffset	= 0x09,
	Qmi8658_Ctrl9_Cmd_GyroHostDeltaOffset	= 0x0A,
	Qmi8658_Ctrl9_Cmd_EnableExtReset		= 0x0B,
	Qmi8658_Ctrl9_Cmd_EnableTap			= 0x0C,	
	Qmi8658_Ctrl9_Cmd_EnablePedometer	= 0x0D,
	
	Qmi8658_Ctrl9_Cmd_Motion				= 0x0E,
	Qmi8658_Ctrl9_Cmd_CopyUsid				= 0x10,
	Qmi8658_Ctrl9_Cmd_SetRpu				= 0x11,
	Qmi8658_Ctrl9_Cmd_Dbg_WoM_Data_Enable	= 0xF8,
};


enum Qmi8658_LpfConfig
{
	Qmi8658Lpf_Disable, /*!< \brief Disable low pass filter. */
	Qmi8658Lpf_Enable   /*!< \brief Enable low pass filter. */
};

enum Qmi8658_HpfConfig
{
	Qmi8658Hpf_Disable, /*!< \brief Disable high pass filter. */
	Qmi8658Hpf_Enable   /*!< \brief Enable high pass filter. */
};

enum Qmi8658_StConfig
{
	Qmi8658St_Disable, /*!< \brief Disable high pass filter. */
	Qmi8658St_Enable   /*!< \brief Enable high pass filter. */
};

enum Qmi8658_LpfMode
{
	A_LSP_MODE_0 = 0x00<<1,
	A_LSP_MODE_1 = 0x01<<1,
	A_LSP_MODE_2 = 0x02<<1,
	A_LSP_MODE_3 = 0x03<<1,
	
	G_LSP_MODE_0 = 0x00<<5,
	G_LSP_MODE_1 = 0x01<<5,
	G_LSP_MODE_2 = 0x02<<5,
	G_LSP_MODE_3 = 0x03<<5
};

enum Qmi8658_AccRange
{
	Qmi8658AccRange_2g = 0x00 << 4, /*!< \brief +/- 2g range */
	Qmi8658AccRange_4g = 0x01 << 4, /*!< \brief +/- 4g range */
	Qmi8658AccRange_8g = 0x02 << 4, /*!< \brief +/- 8g range */
	Qmi8658AccRange_16g = 0x03 << 4 /*!< \brief +/- 16g range */
};


enum Qmi8658_AccOdr
{
	Qmi8658AccOdr_8000Hz = 0x00,  /*!< \brief High resolution 8000Hz output rate. */
	Qmi8658AccOdr_4000Hz = 0x01,  /*!< \brief High resolution 4000Hz output rate. */
	Qmi8658AccOdr_2000Hz = 0x02,  /*!< \brief High resolution 2000Hz output rate. */
	Qmi8658AccOdr_1000Hz = 0x03,  /*!< \brief High resolution 1000Hz output rate. */
	Qmi8658AccOdr_500Hz = 0x04,  /*!< \brief High resolution 500Hz output rate. */
	Qmi8658AccOdr_250Hz = 0x05, /*!< \brief High resolution 250Hz output rate. */
	Qmi8658AccOdr_125Hz = 0x06, /*!< \brief High resolution 125Hz output rate. */
	Qmi8658AccOdr_62_5Hz = 0x07, /*!< \brief High resolution 62.5Hz output rate. */
	Qmi8658AccOdr_31_25Hz = 0x08,  /*!< \brief High resolution 31.25Hz output rate. */
	Qmi8658AccOdr_LowPower_128Hz = 0x0c, /*!< \brief Low power 128Hz output rate. */
	Qmi8658AccOdr_LowPower_21Hz = 0x0d,  /*!< \brief Low power 21Hz output rate. */
	Qmi8658AccOdr_LowPower_11Hz = 0x0e,  /*!< \brief Low power 11Hz output rate. */
	Qmi8658AccOdr_LowPower_3Hz = 0x0f    /*!< \brief Low power 3Hz output rate. */
};

enum Qmi8658_GyrRange
{
#if 0
	Qmi8658GyrRange_32dps = 0 << 4,   /*!< \brief +-32 degrees per second. */
	Qmi8658GyrRange_64dps = 1 << 4,   /*!< \brief +-64 degrees per second. */
	Qmi8658GyrRange_128dps = 2 << 4,  /*!< \brief +-128 degrees per second. */
	Qmi8658GyrRange_256dps = 3 << 4,  /*!< \brief +-256 degrees per second. */
	Qmi8658GyrRange_512dps = 4 << 4,  /*!< \brief +-512 degrees per second. */
	Qmi8658GyrRange_1024dps = 5 << 4, /*!< \brief +-1024 degrees per second. */
	Qmi8658GyrRange_2048dps = 6 << 4, /*!< \brief +-2048 degrees per second. */
	Qmi8658GyrRange_4096dps = 7 << 4  /*!< \brief +-2560 degrees per second. */
#else	
	Qmi8658GyrRange_16dps = 0 << 4,   /*!< \brief +-32 degrees per second. */
	Qmi8658GyrRange_32dps = 1 << 4,   /*!< \brief +-32 degrees per second. */
	Qmi8658GyrRange_64dps = 2 << 4,   /*!< \brief +-64 degrees per second. */
	Qmi8658GyrRange_128dps = 3 << 4,  /*!< \brief +-128 degrees per second. */
	Qmi8658GyrRange_256dps = 4 << 4,  /*!< \brief +-256 degrees per second. */
	Qmi8658GyrRange_512dps = 5 << 4,  /*!< \brief +-512 degrees per second. */
	Qmi8658GyrRange_1024dps = 6 << 4, /*!< \brief +-1024 degrees per second. */
	Qmi8658GyrRange_2048dps = 7 << 4, /*!< \brief +-2048 degrees per second. */
#endif
};

/*!
 * \brief Gyroscope output rate configuration.
 */
enum Qmi8658_GyrOdr
{
	Qmi8658GyrOdr_8000Hz = 0x00,  /*!< \brief High resolution 8000Hz output rate. */
	Qmi8658GyrOdr_4000Hz = 0x01,  /*!< \brief High resolution 4000Hz output rate. */
	Qmi8658GyrOdr_2000Hz = 0x02,  /*!< \brief High resolution 2000Hz output rate. */
	Qmi8658GyrOdr_1000Hz = 0x03,	/*!< \brief High resolution 1000Hz output rate. */
	Qmi8658GyrOdr_500Hz	= 0x04,	/*!< \brief High resolution 500Hz output rate. */
	Qmi8658GyrOdr_250Hz	= 0x05,	/*!< \brief High resolution 250Hz output rate. */
	Qmi8658GyrOdr_125Hz	= 0x06,	/*!< \brief High resolution 125Hz output rate. */
	Qmi8658GyrOdr_62_5Hz 	= 0x07,	/*!< \brief High resolution 62.5Hz output rate. */
	Qmi8658GyrOdr_31_25Hz	= 0x08	/*!< \brief High resolution 31.25Hz output rate. */
};

enum Qmi8658_AeOdr
{
	Qmi8658AeOdr_1Hz = 0x00,  /*!< \brief 1Hz output rate. */
	Qmi8658AeOdr_2Hz = 0x01,  /*!< \brief 2Hz output rate. */
	Qmi8658AeOdr_4Hz = 0x02,  /*!< \brief 4Hz output rate. */
	Qmi8658AeOdr_8Hz = 0x03,  /*!< \brief 8Hz output rate. */
	Qmi8658AeOdr_16Hz = 0x04, /*!< \brief 16Hz output rate. */
	Qmi8658AeOdr_32Hz = 0x05, /*!< \brief 32Hz output rate. */
	Qmi8658AeOdr_64Hz = 0x06,  /*!< \brief 64Hz output rate. */
	Qmi8658AeOdr_128Hz = 0x07,  /*!< \brief 128Hz output rate. */
	/*!
	 * \brief Motion on demand mode.
	 *
	 * In motion on demand mode the application can trigger AttitudeEngine
	 * output samples as necessary. This allows the AttitudeEngine to be
	 * synchronized with external data sources.
	 *
	 * When in Motion on Demand mode the application should request new data
	 * by calling the Qmi8658_requestAttitudeEngineData() function. The
	 * AttitudeEngine will respond with a data ready event (INT2) when the
	 * data is available to be read.
	 */
	Qmi8658AeOdr_motionOnDemand = 128
};

enum Qmi8658_MagOdr
{
	Qmi8658MagOdr_1000Hz = 0x00,   /*!< \brief 1000Hz output rate. */
	Qmi8658MagOdr_500Hz = 0x01,	/*!< \brief 500Hz output rate. */
	Qmi8658MagOdr_250Hz = 0x02,	/*!< \brief 250Hz output rate. */
	Qmi8658MagOdr_125Hz = 0x03,	/*!< \brief 125Hz output rate. */
	Qmi8658MagOdr_62_5Hz = 0x04,	/*!< \brief 62.5Hz output rate. */
	Qmi8658MagOdr_31_25Hz = 0x05	/*!< \brief 31.25Hz output rate. */
};

enum Qmi8658_MagDev
{
	MagDev_AKM09918 = (0 << 3), /*!< \brief AKM09918. */
};

enum Qmi8658_AccUnit
{
	Qmi8658AccUnit_g,  /*!< \brief Accelerometer output in terms of g (9.81m/s^2). */
	Qmi8658AccUnit_ms2 /*!< \brief Accelerometer output in terms of m/s^2. */
};

enum Qmi8658_GyrUnit
{
	Qmi8658GyrUnit_dps, /*!< \brief Gyroscope output in degrees/s. */
	Qmi8658GyrUnit_rads /*!< \brief Gyroscope output in rad/s. */
};

enum Qmi8658_FifoMode
{
	Qmi8658_Fifo_Bypass = 0,
	Qmi8658_Fifo_Fifo = 1,
	Qmi8658_Fifo_Stream = 2,	
	Qmi8658_Fifo_StreamToFifo = 3
};


enum Qmi8658_FifoWmkLevel
{
	Qmi8658_Fifo_WmkEmpty = 		(0 << 4),
	Qmi8658_Fifo_WmkOneQuarter =	(1 << 4),
	Qmi8658_Fifo_WmkHalf =			(2 << 4),
	Qmi8658_Fifo_WmkThreeQuarters =	(3 << 4)
};

enum Qmi8658_FifoSize
{
	Qmi8658_Fifo_16 = (0 << 2),
	Qmi8658_Fifo_32 = (1 << 2),
	Qmi8658_Fifo_64 = (2 << 2),
	Qmi8658_Fifo_128 = (3 << 2)
};

struct Qmi8658_offsetCalibration
{
	enum Qmi8658_AccUnit accUnit;
	float accOffset[3];
	enum Qmi8658_GyrUnit gyrUnit;
	float gyrOffset[3];
};

struct Qmi8658_sensitivityCalibration
{
	float accSensitivity[3];
	float gyrSensitivity[3];
};

enum Qmi8658_Interrupt
{
	Qmi8658_Int1_low = (0 << 6),
	Qmi8658_Int2_low = (1 << 6),
	Qmi8658_Int1_high = (2 << 6),
	Qmi8658_Int2_high = (3 << 6)
};

enum Qmi8658_InterruptState
{
	Qmi8658State_high = (1 << 7),
	Qmi8658State_low  = (0 << 7)
};

enum Qmi8658_WakeOnMotionThreshold
{
	Qmi8658WomThreshold_off  = 0,
	Qmi8658WomThreshold_low  = 32,
	Qmi8658WomThreshold_mid = 128,
	Qmi8658WomThreshold_high = 255
};

typedef enum Qmi8658_fifo_format {
    QMI8658_FORMAT_EMPTY,
    QMI8658_FORMAT_ACCEL_6_BYTES,
    QMI8658_FORMAT_GYRO_6_BYTES,
    QMI8658_FORMAT_12_BYTES,

    QMI8658_FORMAT_UNKNOWN = 0xff,
} Qmi8658_fifo_format;

struct Qmi8658Config
{
	unsigned char inputSelection;
	enum Qmi8658_AccRange accRange;
	enum Qmi8658_AccOdr accOdr;
	enum Qmi8658_GyrRange gyrRange;
	enum Qmi8658_GyrOdr gyrOdr;
	enum Qmi8658_AeOdr aeOdr;
	enum Qmi8658_MagOdr magOdr;
	enum Qmi8658_MagDev magDev;
#if defined(QMI8658_USE_FIFO)
	unsigned char			fifo_ctrl;
	unsigned char			fifo_fss;
	Qmi8658_fifo_format		fifo_format;
	//unsigned char	fifo_status;
#endif
};

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *             Function Declarations
 ******************************************************/
void MX_I2C1_Init(void);
void MX_SPI1_Init(void);
void QMI8658_Sensor_Test(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _QMI8658_H_ */
