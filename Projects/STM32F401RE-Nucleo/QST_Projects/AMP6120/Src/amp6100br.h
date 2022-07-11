/*
********************************************************************************
*
* File : amp6100br.h
*
* Date : 2022/05/04
*
* Revision : 1.0.0
*
* Usage: The header file for the sensor AMP6100 configuration and operation interface.
*
*******************************************************************************/
#ifndef __AMP6100BR_H
#define __AMP6100BR_H

#include "i2c_interface.h"

#define AMP6100BR_DEV_ID        0x78

/* Sensor operating status information */
typedef union{
    uint8_t all;
    struct{
        uint8_t bit0:1;
        uint8_t bit1:1;
        uint8_t nvmErr:1;
        uint8_t mode:1;
        uint8_t bit4:1;
        uint8_t busy:1;
        uint8_t adcPwr:1;
        uint8_t bit7:1;
    }bit;
}
amp_stus_t;

/* AEF format define */
typedef union{
    uint16_t all;
    struct{
        uint16_t gain_stage1:2;
        uint16_t gain_stage2:3;
        uint16_t gain_polarity:1;
        uint16_t clk_divider:2;
        uint16_t A2D_offset:3;
        uint16_t osr_p:3;
        uint16_t osr_t:2;
    }bit;
}
amp_afe_t;

/* NVM register define */
#define MAX_REG_NUMBER      32
typedef union{
    uint16_t array[MAX_REG_NUMBER];
}
amp_register_t;

#define MAX_COT_NUMBER      3
#define MAX_COP_NUMBER      8

typedef struct{
    uint8_t              portID;
    uint8_t              started;
    amp_stus_t      stus;
    amp_afe_t       afe_default;
    amp_register_t* pRegister;
    uint32_t             co_t_array[MAX_COT_NUMBER];
    uint32_t             co_p_array[MAX_COP_NUMBER];
    
    uint16_t             osr_p;
    uint16_t             osr_t;
    uint8_t              iir_flg;
    float           odr;            /* Data output rate, unit:1Hz */
    
    int16_t             tempDat_max;    /* real temperature max data, unit: 0.1 cnetigrade */
    int16_t             tempDat_min;
    uint32_t             presDat_max;    /* real pressure max data, unit: 1hPa */
    uint32_t             presDat_min;
    
    int16_t             temp_rt;
    uint32_t             pres_rt;
    
    uint16_t             temp_raw;
    uint32_t             pres_raw;
    
    uint8_t              cmd_pres;
    //uint8_t              cmd_temp;
}
amp_object_t;
extern amp_object_t amp_entity;

/** Global function Declarations */
uint8_t amp_write_cmd(uint8_t *pdat, uint8_t len);
uint8_t amp_read_stus(void);
uint8_t amp_read_bytes(uint8_t *dst, uint8_t len);
uint8_t amp_read_reg(uint8_t addr, uint16_t *reg);
uint8_t amp_read_PresDat(uint32_t *pres);
uint16_t amp_read_TempDat(uint16_t *temp);

void Amp_Control_Req(uint8_t cmd, uint8_t osr_p, uint8_t osr_t);
void Amp_ADC_Start(uint8_t en);
void Amp_IIR_Enable(uint8_t en);
void Amp_ODR_Set(uint16_t freq);

void SensorSampleOutput(amp_object_t *entity, uint16_t timebyms);
void AmpSensorInit(void);
void amp6100br_Init(void);

#endif /* End of amp6100br include */

/*******************END OF FILE******************************************************************************/
