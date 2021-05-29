
#include <stdio.h>
#include "flash.h"

#define sec4_rang 		65535
#define sec5_rang 		131071

#define Start_Addr_sec3 0x0800c000
#define Start_Addr_sec4 0x08010000
#define Start_Addr_sec5 0x08020000 
#define Start_Addr_sec6 0x08040000 
#define Start_Addr_sec7 0x08060000
/*
sector 3  0x800c000   -    0x0800FFFF  16K
sector 4  0x8010000   -    0x0801FFFF  64K
sector 5  0x8020000   -    0x0803FFFF  128K  131072 bytes
sector 6  0x8040000   -    0x0805FFFF  128K  131072 bytes
sector 7  0x8060000   -    0x0807FFFF  128K  131072 bytes
*/


union uint32_byte1
{
  uint32_t u32_data;
  uint8_t byte[4];
} uint32_to_byte1;

union byte1_uint32
{
  uint8_t byte[4];
  uint32_t u32_data;
}	byte_to_u32data1;





uint32_t store_addr[4]; //len 16bytes
uint16_t store_count = 0;  //store status every 2048 bytes
uint32_t sec_addr[4] = {Start_Addr_sec4,Start_Addr_sec5,Start_Addr_sec6,Start_Addr_sec7};
uint8_t write_sector[4] = {FLASH_SECTOR_4,FLASH_SECTOR_5,FLASH_SECTOR_6,FLASH_SECTOR_7};

uint8_t  sec_index = 0; // len 4bytes
uint8_t  rsec_index = 0;

uint32_t write_addr = Start_Addr_sec4;
uint32_t read_addr = Start_Addr_sec4;

uint32_t sec_rang = sec4_rang;
uint32_t rsec_rang = sec4_rang;

uint8_t  erase_flag = 0;



void Erase_data(void)
{
	erase_flag = 0;
	sec_rang = sec4_rang;
	rsec_rang = sec4_rang;
	write_addr = Start_Addr_sec4;
	read_addr = Start_Addr_sec4;
	sec_index = 0;
	rsec_index = 0;
	store_addr[0] = 0;
	store_addr[1] = 0;
	store_addr[2] = 0;
	store_addr[3] = 0;
	
}



void Flash_Read(uint32_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead)   	
{
	uint16_t i;
	for(i = 0; i < NumToRead; i++)
	{
		pBuffer[i] = *(__IO uint8_t *)ReadAddr;
		ReadAddr += 1;	
	}
}

/*


*/
void Flash_Write1(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite)
{	
	HAL_StatusTypeDef FlashStatus;  
	uint32_t endaddr = 0;   
	
	HAL_FLASH_Unlock();	//解锁  
	endaddr = WriteAddr + NumToWrite;

	FLASH_Erase_Sector(FLASH_SECTOR_3, FLASH_VOLTAGE_RANGE_3);

	
	FlashStatus = FLASH_WaitForLastOperation(50000);
	if(FlashStatus == HAL_OK)
	{
		while(WriteAddr < endaddr)
		{
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, WriteAddr, *pBuffer) != HAL_OK)
			{
				break;
			}
			WriteAddr += 1;
			pBuffer++;
		}
	}
	
	HAL_FLASH_Lock();		//上锁
}



void Flash_Write(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite)
{	
	HAL_StatusTypeDef FlashStatus;  
	uint32_t endaddr = 0;   
	
	HAL_FLASH_Unlock();	//解锁  
	endaddr = WriteAddr + NumToWrite;
	if(erase_flag == 0)
	{
		FLASH_Erase_Sector(write_sector[sec_index], FLASH_VOLTAGE_RANGE_3);
		erase_flag = 1;
	}
	
	FlashStatus = FLASH_WaitForLastOperation(50000);
	if(FlashStatus == HAL_OK)
	{
		while(WriteAddr < endaddr)
		{
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, WriteAddr, *pBuffer) != HAL_OK)
			{
				break;
			}
			WriteAddr += 1;
			pBuffer++;
		}
	}
	
	HAL_FLASH_Lock();		//上锁
}


uint8_t Check_data(void)
{
	uint8_t buf[21];
	//uint8_t tmp_index = 0,i = 0;
	Flash_Read(Start_Addr_sec3,buf,21);
	
	if(buf[0] == 1)
	{
		byte_to_u32data1.byte[0] = buf[17];
		byte_to_u32data1.byte[1] = buf[18];
		byte_to_u32data1.byte[2] = buf[19];
		byte_to_u32data1.byte[3] = buf[20];
		
		sec_index = byte_to_u32data1.u32_data;
		byte_to_u32data1.byte[0] = buf[1];
		byte_to_u32data1.byte[1] = buf[2];
		byte_to_u32data1.byte[2] = buf[3];
		byte_to_u32data1.byte[3] = buf[4];
		store_addr[0] = byte_to_u32data1.u32_data;

		byte_to_u32data1.byte[0] = buf[5];
		byte_to_u32data1.byte[1] = buf[6];
		byte_to_u32data1.byte[2] = buf[7];
		byte_to_u32data1.byte[3] = buf[8];
		store_addr[1] = byte_to_u32data1.u32_data;
		
		byte_to_u32data1.byte[0] = buf[9];
		byte_to_u32data1.byte[1] = buf[10];
		byte_to_u32data1.byte[2] = buf[11];
		byte_to_u32data1.byte[3] = buf[12];
		store_addr[2] = byte_to_u32data1.u32_data;
		
		byte_to_u32data1.byte[0] = buf[13];
		byte_to_u32data1.byte[1] = buf[14];
		byte_to_u32data1.byte[2] = buf[15];
		byte_to_u32data1.byte[3] = buf[16];
		store_addr[3] = byte_to_u32data1.u32_data;
		
		printf("\n store_flag:%d,",buf[0]);
		printf("sec_index:%d,store_addr[0]:0x%x,store_addr[1]:0x%x,store_addr[2]:0x%x,store_addr[3]:0x%x\n",sec_index,store_addr[0],store_addr[1],store_addr[2],store_addr[3]);	
		return 1;
	}
	
	return 0;
	
}


void Store_status(void)
{
	uint8_t buf[21];
	
	
	uint32_to_byte1.u32_data = store_addr[0];
	buf[0] = 1;
	buf[1] = uint32_to_byte1.byte[0];
	buf[2] = uint32_to_byte1.byte[1];
	buf[3] = uint32_to_byte1.byte[2];
	buf[4] = uint32_to_byte1.byte[3];
	
	uint32_to_byte1.u32_data = store_addr[1];
	buf[5] = uint32_to_byte1.byte[0];
	buf[6] = uint32_to_byte1.byte[1];
	buf[7] = uint32_to_byte1.byte[2];
	buf[8] = uint32_to_byte1.byte[3];
	
	uint32_to_byte1.u32_data = store_addr[2];
	buf[9] = uint32_to_byte1.byte[0];
	buf[10] = uint32_to_byte1.byte[1];
	buf[11] = uint32_to_byte1.byte[2];
	buf[12] = uint32_to_byte1.byte[3];
	
	uint32_to_byte1.u32_data = store_addr[3];
	buf[13] = uint32_to_byte1.byte[0];
	buf[14] = uint32_to_byte1.byte[1];
	buf[15] = uint32_to_byte1.byte[2];
	buf[16] = uint32_to_byte1.byte[3];
	
	uint32_to_byte1.u32_data = sec_index;
	buf[17] = uint32_to_byte1.byte[0];
	buf[18] = uint32_to_byte1.byte[1];
	buf[19] = uint32_to_byte1.byte[2];
	buf[20] = uint32_to_byte1.byte[3];
	
	Flash_Write1(Start_Addr_sec3,buf,21);
	printf("\n store_flag:%d,",buf[0]);
	printf("sec_index:%d,store_addr[0]:0x%x,store_addr[1]:0x%x,store_addr[2]:0x%x,store_addr[3]:0x%x\n",sec_index,store_addr[0],store_addr[1],store_addr[2],store_addr[3]);	
}


uint8_t tmp_d[4];
uint8_t Store_Data(uint8_t *data,uint8_t len)
{
	tmp_d[0] = data[0];
	tmp_d[1] = data[1];
	tmp_d[2] = data[2];
	tmp_d[3] = data[3];
	
	if((write_addr + len - sec_rang) <= sec_addr[sec_index])
	{
		
		

		Flash_Write(write_addr,data,len);
		write_addr+=len;
		store_addr[sec_index] = write_addr;
		
		store_count+=len;
		if(store_count>= 2048)
		{
			
			Store_status();
			store_count = 0;
		}
		return 1;
		
	}
	else
	{
		if(sec_index < 4)
		{
			
			printf("addr:%d,sec_index:%d\n",write_addr,sec_index);
			sec_index++;
			sec_rang = sec5_rang;
			write_addr = sec_addr[sec_index];
			erase_flag = 0; //clear flag
			
			
			Flash_Write(write_addr,data,len);
			write_addr+=len;
			store_addr[sec_index] = write_addr;
			
			store_count+=len;
			if(store_count>= 2048)
			{
				Store_status();
				store_count = 0;
			}
			return 1;
		}
		else
		{
			return 0;
		}
	}
	
	//return 0; 
}
	


uint8_t Read_Data(uint8_t *data)
{
	//if(write_addr == Start_Addr_sec4) return 0;

	uint16_t tmp_len = 1024;
	if((read_addr + 1024) >= store_addr[rsec_index])
	{
		if((read_addr < store_addr[rsec_index]))
		{
			tmp_len = store_addr[rsec_index]  -  read_addr;
			Flash_Read(read_addr, data, tmp_len);
			read_addr += tmp_len;	
			printf("\n read_addr1:0x%x,rsec_index:%d\n",read_addr,rsec_index);
			return 1;
		}
		
		if(rsec_index < sec_index)
		{
			rsec_index++;
			read_addr = sec_addr[rsec_index];
			rsec_rang = sec5_rang;
			Flash_Read(read_addr, data, tmp_len);
			read_addr += tmp_len;	
			printf("\n read_addr2:0x%x,rsec_index:%d\n",read_addr,rsec_index);
			return 1;		
		}
	}
	else
	{
		Flash_Read(read_addr, data, tmp_len);
		read_addr += tmp_len;	
		printf("\n read_addr3:0x%x,rsec_index:%d\n",read_addr,rsec_index);
		return 1;
	}
	return 0;
	
}
