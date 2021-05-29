
#ifndef _FLASH_H
#define _FLASH_H

#include "stm32f4xx_hal.h"

//void ReadCalibrationFromFile(void);
void Flash_Read(uint32_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead);
void Flash_Write(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite);
void Erase_data(void);
uint8_t Check_data(void);
uint8_t Store_Data(uint8_t *data,uint8_t len);
uint8_t Read_Data(uint8_t *data);

#endif
