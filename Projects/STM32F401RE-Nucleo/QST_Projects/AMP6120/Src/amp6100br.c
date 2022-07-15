/*
********************************************************************************
*
* File : amp6100br.c
*
* Date : 2022/05/04
*
* Revision : 1.0.0
*
* Usage: The sensor AMP6100 configuration and operation interface
*
*******************************************************************************/
/******** Includes ************************************************************/
#include "main.h"
#include "string.h"
#include "stdbool.h"
#include "i2c_interface.h"
#include "amp6100br.h"

//extern void SendSampleData(void);

/******** Private definitions *************************************************/
/******** Private function declarations ***************************************/

/******** Variables ***********************************************************/
amp_register_t  amp_Register;
amp_object_t    amp_entity;


/******** Private function declarations ***************************************/

uint8_t amp_read_bytes(uint8_t *pData, uint8_t size)
{
  HAL_StatusTypeDef ret;
  uint8_t dev = amp_entity.portID;//AMP6100BR_DEV_ID;
  ret = BSP_I2C_Read(dev, pData, size);
  //printf("dev(0x%x) read:%d\n", dev, ret);
  return ret == HAL_OK ? true : false;
}

/*
 ** PRIVATE FUNCTION: amp_write_cmd()
 *
 *  DESCRIPTION:
 *      write the request command to sensor thru iic bus.
 *
 *  PARAMETERS:
 *      *pData: the pointer to the command.
 *      size: the length of command in bytes.
 *             
 *  RETURNS:
 *      None.
 */  
uint8_t amp_write_cmd(uint8_t *pData, uint8_t size)
{
  HAL_StatusTypeDef ret;
  uint8_t dev = amp_entity.portID;//AMP6100BR_DEV_ID;

  ret = BSP_I2C_Write(dev, pData, size);
  //printf("dev(0x%x)write(%d):0x%x, ret:%d\n", dev, size, *pData, ret);
  return ret == HAL_OK ? true : false;
}

/*
 ** PRIVATE FUNCTION: amp_read_stus()
 *
 */  
uint8_t amp_read_stus(void)
{  
    uint8_t dev = amp_entity.portID;
    uint8_t stus;
    BSP_I2C_Read(dev, &stus, 1);
    return stus;
}

/*
 ** PRIVATE FUNCTION: amp_read_reg()
 *
 */  
uint8_t amp_read_reg(uint8_t addr, uint16_t *reg)
{
    uint8_t ret = false;
    uint8_t adr = addr;
    uint8_t buf[3];
    
    ret = amp_write_cmd(&adr, 1);
    printf("cmd:%d\n", ret);
    
    if (ret == true){
        ret = amp_read_bytes(buf, 2);
        printf("read:%d\n", ret);
        
        if (ret == true){
            *reg = (((uint16_t)buf[1])<<8) + buf[2];
        }
    }
    
    return ret;
}
/*
 ** PRIVATE FUNCTION: amp_read_PresDat()
 *
 */  
uint8_t amp_read_PresDat(uint32_t *pres)
{
    uint8_t buf[5];
    
    if (amp_read_bytes(buf, 4)){
        *pres = ((uint32_t)buf[1]<<16) + ((uint32_t)buf[2]<<8) + buf[3];
        return true;
    }
    
    return false;
}
/*
 ** PRIVATE FUNCTION: amp_read_ADResult()
 *
 */  
uint8_t amp_read_ADResult(uint8_t *pDat, uint8_t size)
{
    if (amp_read_bytes(pDat, size)){
        return true;
    }

    return false;
}

/*
 ** PRIVATE FUNCTION: amp_read_TempDat()
 *
 */ 
uint16_t amp_read_TempDat(uint16_t *temp)
{
    uint8_t buf[5];

    if (amp_read_bytes(buf, 4)){
        *temp = ((uint16_t)buf[2]<<8) + buf[3];
        return true;
    }

    return false;
}

/*
 ** PRIVATE FUNCTION: TempDat_Convert()
 *
 */ 
int16_t TempDat_Convert(uint16_t raw, uint8_t iir, int16_t temp_max, int16_t temp_min)
{
    int16_t tmp = 0;
    
    if (iir){
        tmp = ((int32_t)raw * (temp_max - temp_min) >> 16) + temp_min;
    }
    else{
        tmp = ((int32_t)raw * (temp_max - temp_min) >> 16) + temp_min;
    }
    
    tmp  = (tmp > temp_max) ? temp_max : ((tmp < temp_min) ? temp_min : tmp);
    
    return tmp;
}



/*
 ** PRIVATE FUNCTION: iir_prs_filter()
 *
 */ 
uint32_t raw_hisotry = 0xFFFFFFFF;
uint32_t iir_prs_filter(uint32_t raw, uint8_t iir)
{
	if (iir !=0 && raw_hisotry != 0xFFFFFFFF)
	{
		raw = (raw + (iir - 1) * raw_hisotry) / iir;
	}
	raw_hisotry = raw;
    return raw;
}

/*
 ** PRIVATE FUNCTION: PresDat_Convert()
 *
 */ 
uint32_t PresDat_Convert(uint32_t raw, uint8_t iir, int32_t pres_max, int32_t pres_min)
{
    uint32_t tmp = 0;
    
    if (iir){
        uint64_t tmp64;

        tmp64 = iir_prs_filter(raw, iir);
        tmp = ((uint64_t)tmp64 * (pres_max - pres_min) >> 24) + pres_min;
    }
    else{
        tmp = ((uint64_t)raw * (pres_max - pres_min) >> 24) + pres_min;
    }
    
    tmp  = (tmp > pres_max) ? pres_max : ((tmp < pres_min) ? pres_min : tmp);
    
    return (uint32_t)tmp;
}
/*
 ** PUBLIC FUNCTION: Amp_Control_Req()
 *
 */ 
void Amp_Control_Req(uint8_t cmd, uint8_t osr_p, uint8_t osr_t)
{
    uint8_t t_cmd = cmd;
    switch(cmd){
    case 0xAC:
        amp_entity.cmd_pres = t_cmd;
        break;
    default:
        if ((cmd&0xB0) == 0xB0){
            amp_entity.cmd_pres = t_cmd;
            switch((cmd>>3)&0x01){
            case 0:
                amp_entity.osr_t = 4;
                break;
            default:
                amp_entity.osr_t = 8;
            }
            switch(cmd&0x07){
            case 0: amp_entity.osr_p = 128; break;
            case 1: amp_entity.osr_p = 64; break;
            case 2: amp_entity.osr_p = 32; break;
            case 3: amp_entity.osr_p = 16; break;
            case 4: amp_entity.osr_p = 8; break;
            case 5: amp_entity.osr_p = 4; break;
            case 6: amp_entity.osr_p = 2; break;
            case 7: amp_entity.osr_p = 1; break;                
            default: break;
            }
        }
        break;
    }
}
/*
 ** PRIVATE FUNCTION: osr_config_set()
 *
 */ 
uint8_t osr_config_set(uint8_t osr_p, uint8_t osr_t)
{
	uint8_t cmd = 0xB0;
	
	if (osr_t == 8){
		cmd |= 0x08;
	}
	switch(osr_p){
		case 1:  cmd |= 0x07; break;
		case 2:  cmd |= 0x06; break;
		case 4:  cmd |= 0x05; break;
		case 8:  cmd |= 0x04; break;
		case 16: cmd |= 0x03; break;
		case 32: cmd |= 0x02; break;
		case 64: cmd |= 0x01; break;
		default: break;
	}
	return cmd;
}

/*
 ** PUBLIC FUNCTION: Amp_ADC_Start()
 *
 */ 
void Amp_ADC_Start(uint8_t en)
{
    if (en)
        amp_entity.started = 1;
    else
        amp_entity.started = 0;
}

/*
 ** PUBLIC FUNCTION: Amp_IIR_Enable()
 *
 *  DESCRIPTION:
 *      config the I2C pin.
 *
 *  PARAMETERS:
 *      en: 0~16, 0: disable, !0: filterfactor
 *             
 *  RETURNS:
 *      None.
 */ 
void Amp_IIR_Enable(uint8_t en)
{
    amp_entity.iir_flg = en;
}

/*
 ** PUBLIC FUNCTION: Amp_ODR_Set()
 *
 */ 
void Amp_ODR_Set(uint16_t freq)
{
    if (freq >=1 && freq <= 100){  // 1~100Hz
        amp_entity.odr = (float)freq;
    }
}

/*
 ** PUBLIC FUNCTION: SensorSampleOutput()
 *
 */ 
uint16_t timeDelay = 0;
uint8_t tmpbuf[10];
void SensorSampleOutput(amp_object_t *entity, uint16_t timebyms)
{
    static uint16_t timer;
    uint16_t cyclebyms = (uint16_t)((float)1000.0/entity->odr) + 1;
    uint16_t raw_temp=0xFFFF;//, timeDelay;
    uint32_t raw_pres=0x00FFFFFF;
    uint8_t cmd;
    //uint32_t tmp32;
    uint32_t sampleTimestamp;
    
    if (entity->started == 0){
        return;
    }
    
    timer++;
    // printf("Time:%ld, timer:%d, timebyms:%d, cycle:%d\n",
    //          HAL_GetTick(), timer, timebyms, cyclebyms);
    if (timer*timebyms >= cyclebyms){
        timer = 0;
        cmd = entity->cmd_pres;
        
        /* 1. Start ADC */
        tmpbuf[0] = cmd;
        amp_write_cmd(tmpbuf, 1);

        /* 2. Waiting */
        HAL_Delay(2);//vTaskDelay(2);//delay_us(5000);
        for(timeDelay = 0; timeDelay < 0x02FF; timeDelay++){
            amp_stus_t stus;
            stus.all = amp_read_stus();
            
            entity->stus.all = stus.all;
            if (stus.bit.busy == 0){
                break;
            }
        }

        sampleTimestamp = HAL_GetTick();

        /* 3. Read AD result */
        if (amp_read_ADResult(tmpbuf, 6)){
            entity->stus.all = tmpbuf[0];
            raw_pres = ((uint32_t)tmpbuf[1]<<16) + ((uint32_t)tmpbuf[2]<<8) + ((uint32_t)tmpbuf[3]<<0);
            raw_temp = ((uint16_t)tmpbuf[4]<< 8) + ((uint16_t)tmpbuf[5]<<0);
        }
        
        /* 4. Convert raw to real value */
        if (raw_pres!=0x00FFFFFF){
            entity->pres_raw = raw_pres;
            entity->pres_rt = PresDat_Convert(raw_pres, entity->iir_flg, entity->presDat_max, entity->presDat_min);
            entity->pres_raw = raw_hisotry;
        }
        if (raw_temp!=0xFFFF){
            entity->temp_raw = raw_temp;
            entity->temp_rt = TempDat_Convert(raw_temp, entity->iir_flg, entity->tempDat_max, entity->tempDat_min);
        }
#if 0
        //SendSampleData();
#else
        printf("[%02d:%02d:%03d]: pres:%ld, temp:%ld\n", sampleTimestamp/60000, \
                                   (sampleTimestamp%60000)/1000, (sampleTimestamp%60000)%1000, \
                                   entity->pres_rt, entity->temp_rt);
#endif
    }
}

/*
 ** PRIVATE FUNCTION: AmpSensorInit()
 *
 */ 
void AmpSensorInit(void)
{
    amp_object_t *entity;
    
    memset(&amp_Register, 0, sizeof(amp_register_t));
    memset(&amp_entity, 0, sizeof(amp_object_t));
    
    entity = &amp_entity;
    
    entity->portID = AMP6100BR_DEV_ID;
    entity->pRegister = &amp_Register;
    
    entity->osr_p = 16;
    entity->osr_t = 4;
    entity->iir_flg = 1;
    entity->odr = 1.0;       /* 1Hz */
    
    entity->tempDat_max = 1500;
    entity->tempDat_min = -400;
    entity->presDat_max = 120000;
    entity->presDat_min = 20000;
    
    entity->cmd_pres = osr_config_set(entity->osr_p, entity->osr_t);
    
    for(uint8_t i = 0; i < MAX_REG_NUMBER; i++){
        //amp_read_reg(i, &amp_Register.array[i]);
        //printf("0x%x%s", amp_Register.array[i], (i+1)%4?",":"\n");
    }
    entity->afe_default.all = entity->pRegister->array[14];
    
    entity->started = 1;    // only for debug
}


/*******************END OF FILE************************************************/
