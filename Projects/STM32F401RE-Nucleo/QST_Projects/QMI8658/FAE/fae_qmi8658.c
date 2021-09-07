
#include "qmi8658.h"
#include "gpio.h"
#include "error.h"
#include "i2c.h"
#include "log.h"


#define QMI8658_SLAVE_ADDR_L			0x6a
#define QMI8658_SLAVE_ADDR_H			0x6b
#define qmi8658_printf					LOG

//#define QMI8658_UINT_MG_DPS
//extern uint32_t system_ticks;
enum
{
	AXIS_X = 0,
	AXIS_Y = 1,
	AXIS_Z = 2,

	AXIS_TOTAL
};

typedef struct 
{
	short 				sign[AXIS_TOTAL];
	unsigned short 		map[AXIS_TOTAL];
}qst_imu_layout;

static unsigned short acc_lsb_div = 0;
static unsigned short gyro_lsb_div = 0;
static unsigned short ae_q_lsb_div = (1 << 14);
static unsigned short ae_v_lsb_div = (1 << 10);
static unsigned int imu_timestamp = 0;
struct Qmi8658Config qmi8658_config;
static unsigned char qmi8658_slave_addr = QMI8658_SLAVE_ADDR_H;


void* qmi8658_pi2c;
static void* qmi8658_i2c_init(void)
{
	void* pi2c;
	
	hal_i2c_pin_init(I2C_1, P18, P15);
	pi2c = hal_i2c_init(I2C_1,I2C_CLOCK_400K);
	
	qmi8658_pi2c = pi2c;
	
	#ifdef QMI8658_DEBUG
	{
		#ifdef PRINT_LOG
		LOG("I2C init:\t %d\r\n", pi2c);
		#endif
	
	}
	#endif
	return pi2c;
}

static int qmi8658_i2c_deinit(void* pi2c)
{
	int ret;
	
	ret = hal_i2c_deinit(pi2c);
	hal_gpio_pin_init(P18,IE);
	hal_gpio_pin_init(P15,IE);
	hal_gpio_pull_set(P18,FLOATING); 
	hal_gpio_pull_set(P15,FLOATING); 
	
	qmi8658_pi2c = pi2c;	
	return ret;
}


int iic_read_reg(uint16_t DevAddress, uint8_t *pData, uint16_t Size) 
{
	return hal_i2c_read(qmi8658_pi2c, QMI8658_SLAVE_ADDR_H, DevAddress, pData, Size);
}

int iic_write_reg(uint16_t DevAddress, uint8_t *pData, uint16_t Size) 
{
	uint8_t data[2];
	
	data[0] = DevAddress;
	data[1] = *pData;
	hal_i2c_addr_update(qmi8658_pi2c, QMI8658_SLAVE_ADDR_H);
	{
		hal_i2c_tx_start(qmi8658_pi2c);
		hal_i2c_send(qmi8658_pi2c, data, 2);
	}
	return hal_i2c_wait_tx_completed(qmi8658_pi2c);
}


void Qmi8658_delay(int ms)
{
	#if 0
	int i,j;
	for(i=0;i<ms;i++)
	{
		for(j=0;j<1000;j++)
		{
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();

			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();

			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
		}
	}
	#else
	WaitMs(ms);
	#endif
}

void qmi8658_delay_ms(int ms)
{
	#if 0
	int i,j;
	for(i=0;i<ms;i++)
	{
		for(j=0;j<2000;j++)
		{
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();

			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();

			__NOP(); __NOP(); __NOP(); __NOP();
			__NOP(); __NOP(); __NOP(); __NOP();
		}
	}
	#else
		WaitMs(ms);
	#endif
}

#if 0
unsigned char Qmi8658_write_reg(unsigned char reg, unsigned char value)
{
	unsigned char ret=1;
	unsigned int retry = 0;

	while((ret) && (retry++ < 5))
	{
#if defined(QST_USE_SPI)
		ret = qst_fisimu_spi_write(reg,value);
#elif defined(QST_USE_SW_I2C)
		ret = qst_sw_writereg(qmi8658_slave_addr<<1, reg, value);
#else
		ret = mx_i2c1_write(qmi8658_slave_addr<<1, reg, value);
#endif
	}
	return ret;
}

unsigned char Qmi8658_write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
	unsigned char ret=1;	
	unsigned int retry = 0;

	while((ret) && (retry++ < 5))
	{
#if defined(QST_USE_SPI)
		ret = qst_fisimu_spi_write_bytes(reg, value, len);
#elif defined(QST_USE_SW_I2C)
		ret = qst_sw_writeregs(qmi8658_slave_addr<<1, reg, value, len);
#else
		ret = I2C_BufferRead(qmi8658_slave_addr<<1, reg, value, len);
#endif
	}
	return ret;
}

unsigned char Qmi8658_read_reg(unsigned char reg, unsigned char* buf, unsigned short len)
{
	unsigned char ret=1;
	unsigned int retry = 0;

	while((ret) && (retry++ < 5))
	{
#if defined(QST_USE_SPI)
		ret = qst_8658_spi_read(reg, buf, len);
#elif defined(QST_USE_SW_I2C)
		ret = qst_sw_readreg(qmi8658_slave_addr<<1, reg, buf, len);
#else
		ret = mx_i2c1_read(qmi8658_slave_addr<<1, reg, buf, len);
#endif
		//LOG("r %d\n",retry);
	}
	return ret;
}
#else
unsigned char Qmi8658_write_reg(unsigned char reg, unsigned char value)
{
	unsigned char ret=1;
	unsigned int retry = 0;

	while((ret) && (retry++ < 5))
	{

		//ret = mx_i2c1_write(qmi8658_slave_addr<<1, reg, value);
		//ret = (unsigned char)iic_write_reg(reg,(uint8_t*)value,1);
		ret = (unsigned char)iic_write_reg(reg,&value,1);//20210526,liu
		//LOG("w %d\n",retry);
		

	}
	return ret;
}

unsigned char Qmi8658_write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
	unsigned char ret=1;	
	unsigned int retry = 0;

	while((ret) && (retry++ < 5))
	{
		ret = (unsigned char) iic_read_reg(reg,(uint8_t*) value, len);

	}
	return ret;
}

unsigned char Qmi8658_read_reg(unsigned char reg, unsigned char* buf, unsigned short len)
{
	unsigned char ret=1;
	unsigned int retry = 0;

	while((ret) && (retry++ < 5))
	{
		//ret = qst_sw_readreg(qmi8658_slave_addr<<1, reg, buf, len);
		ret =(unsigned char) iic_read_reg(reg, (uint8_t*)buf, len);
		//LOG("r %d\n",retry);
	}
	return ret;
}

#endif

#if 0
static qst_imu_layout imu_map;

void Qmi8658_set_layout(short layout)
{
	if(layout == 0)
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 1)
	{
		imu_map.sign[AXIS_X] = -1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_Y;
		imu_map.map[AXIS_Y] = AXIS_X;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 2)
	{
		imu_map.sign[AXIS_X] = -1;
		imu_map.sign[AXIS_Y] = -1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 3)
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = -1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_Y;
		imu_map.map[AXIS_Y] = AXIS_X;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}	
	else if(layout == 4)
	{
		imu_map.sign[AXIS_X] = -1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = -1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 5)
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = -1;
		imu_map.map[AXIS_X] = AXIS_Y;
		imu_map.map[AXIS_Y] = AXIS_X;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 6)
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = -1;
		imu_map.sign[AXIS_Z] = -1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 7)
	{
		imu_map.sign[AXIS_X] = -1;
		imu_map.sign[AXIS_Y] = -1;
		imu_map.sign[AXIS_Z] = -1;
		imu_map.map[AXIS_X] = AXIS_Y;
		imu_map.map[AXIS_Y] = AXIS_X;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
	else		
	{
		imu_map.sign[AXIS_X] = 1;
		imu_map.sign[AXIS_Y] = 1;
		imu_map.sign[AXIS_Z] = 1;
		imu_map.map[AXIS_X] = AXIS_X;
		imu_map.map[AXIS_Y] = AXIS_Y;
		imu_map.map[AXIS_Z] = AXIS_Z;
	}
}
#endif

void Qmi8658_config_acc(enum Qmi8658_AccRange range, enum Qmi8658_AccOdr odr, enum Qmi8658_LpfConfig lpfEnable, enum Qmi8658_StConfig stEnable)
{
	unsigned char ctl_dada;
	void* pi2c = qmi8658_i2c_init();
	switch(range)
	{
		case Qmi8658AccRange_2g:
			acc_lsb_div = (1<<14);
			break;
		case Qmi8658AccRange_4g:
			acc_lsb_div = (1<<13);
			break;
		case Qmi8658AccRange_8g:
			acc_lsb_div = (1<<12);
			break;
		case Qmi8658AccRange_16g:
			acc_lsb_div = (1<<11);
			break;
		default: 
			range = Qmi8658AccRange_8g;
			acc_lsb_div = (1<<12);
	}
	if(stEnable == Qmi8658St_Enable)
		ctl_dada = (unsigned char)range|(unsigned char)odr|0x80;
	else
		ctl_dada = (unsigned char)range|(unsigned char)odr;
		
	Qmi8658_write_reg(Qmi8658Register_Ctrl2, ctl_dada);
// set LPF & HPF
	qmi8658_printf("a1\n");
	Qmi8658_read_reg(Qmi8658Register_Ctrl5, &ctl_dada,1);
	ctl_dada &= 0xf0;
	if(lpfEnable == Qmi8658Lpf_Enable)
	{
		ctl_dada |= A_LSP_MODE_3;
		ctl_dada |= 0x01;
	}
	else
	{
		ctl_dada &= ~0x01;
	}
	ctl_dada = 0x00;
	Qmi8658_write_reg(Qmi8658Register_Ctrl5,ctl_dada);
	qmi8658_printf("a2\n");
	qmi8658_i2c_deinit(pi2c);
// set LPF & HPF
}

void Qmi8658_config_gyro(enum Qmi8658_GyrRange range, enum Qmi8658_GyrOdr odr, enum Qmi8658_LpfConfig lpfEnable, enum Qmi8658_StConfig stEnable)
{
	// Set the CTRL3 register to configure dynamic range and ODR
	unsigned char ctl_dada; 
	void* pi2c = qmi8658_i2c_init();
	// Store the scale factor for use when processing raw data
	switch (range)
	{
		case Qmi8658GyrRange_16dps:
			gyro_lsb_div = 2048;
			break;			
		case Qmi8658GyrRange_32dps:
			gyro_lsb_div = 1024;
			break;
		case Qmi8658GyrRange_64dps:
			gyro_lsb_div = 512;
			break;
		case Qmi8658GyrRange_128dps:
			gyro_lsb_div = 256;
			break;
		case Qmi8658GyrRange_256dps:
			gyro_lsb_div = 128;
			break;
		case Qmi8658GyrRange_512dps:
			gyro_lsb_div = 64;
			break;
		case Qmi8658GyrRange_1024dps:
			gyro_lsb_div = 32;
			break;
		case Qmi8658GyrRange_2048dps:
			gyro_lsb_div = 16;
			break;
//		case Qmi8658GyrRange_4096dps:
//			gyro_lsb_div = 8;
//			break;
		default: 
			range = Qmi8658GyrRange_512dps;
			gyro_lsb_div = 64;
			break;
	}

	if(stEnable == Qmi8658St_Enable)
		ctl_dada = (unsigned char)range|(unsigned char)odr|0x80;
	else
		ctl_dada = (unsigned char)range | (unsigned char)odr;
	Qmi8658_write_reg(Qmi8658Register_Ctrl3, ctl_dada);

// Conversion from degrees/s to rad/s if necessary
// set LPF & HPF
	Qmi8658_read_reg(Qmi8658Register_Ctrl5, &ctl_dada,1);
	ctl_dada &= 0x0f;
	if(lpfEnable == Qmi8658Lpf_Enable)
	{
		ctl_dada |= G_LSP_MODE_3;
		ctl_dada |= 0x10;
	}
	else
	{
		ctl_dada &= ~0x10;
	}
	ctl_dada = 0x00;
	Qmi8658_write_reg(Qmi8658Register_Ctrl5,ctl_dada);

	qmi8658_i2c_deinit(pi2c);
// set LPF & HPF
}

void Qmi8658_config_mag(enum Qmi8658_MagDev device, enum Qmi8658_MagOdr odr)
{
	void* pi2c = qmi8658_i2c_init();
	Qmi8658_write_reg(Qmi8658Register_Ctrl4, device|odr);	
	qmi8658_i2c_deinit(pi2c);
}

void Qmi8658_config_ae(enum Qmi8658_AeOdr odr)
{
	void* pi2c = qmi8658_i2c_init();
	//Qmi8658_config_acc(Qmi8658AccRange_8g, AccOdr_1000Hz, Lpf_Enable, St_Enable);
	//Qmi8658_config_gyro(Qmi8658GyrRange_2048dps, GyrOdr_1000Hz, Lpf_Enable, St_Enable);
	Qmi8658_config_acc(qmi8658_config.accRange, qmi8658_config.accOdr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
	Qmi8658_config_gyro(qmi8658_config.gyrRange, qmi8658_config.gyrOdr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
	Qmi8658_config_mag(qmi8658_config.magDev, qmi8658_config.magOdr);
	Qmi8658_write_reg(Qmi8658Register_Ctrl6, odr);
	
	qmi8658_i2c_deinit(pi2c);
}

#if defined(QMI8658_USE_FIFO)
#if 0
void Qmi8658_calc_fifo_status_fss(int enable_num, unsigned short fifoSize)
{
	int total_num = 0;

	if(enable_num == 1)
	{
		total_num = fifoSize*1;
		switch(total_num)
		{
			case 16:
				qmi8658_config.fifo_fss = 2;
				break;
			case 32:
				qmi8658_config.fifo_fss = 4;
				break;
			case 64:
				qmi8658_config.fifo_fss = 8;
				break;
			case 128:
				qmi8658_config.fifo_fss = 16;
				break;
			default:
				qmi8658_config.fifo_fss = 0;
				break;
		}		
	}
	else if(enable_num == 2)
	{
		total_num = fifoSize*2;
		switch(total_num)
		{
			case 32:
				qmi8658_config.fifo_fss = 4;
				break;
			case 64:
				qmi8658_config.fifo_fss = 8;
				break;
			case 128:
				qmi8658_config.fifo_fss = 16;
				break;
			case 256:
				qmi8658_config.fifo_fss = 32;
				break;
			default:
				qmi8658_config.fifo_fss = 0;
				break;
		}
	}
}
#endif
#define QMI8658_HANDSHAKE_NEW
#define QMI8658_HANDSHAKE_TO_STATUS
void Qmi8658_send_ctl9cmd(enum Qmi8658_Ctrl9Command cmd)    //cmd=0x0d
{
	unsigned char	status1 = 0x00;
	unsigned short count=0;
	//void* pi2c = qmi8658_i2c_init();
	Qmi8658_write_reg(Qmi8658Register_Ctrl9, (unsigned char)cmd);	// write commond to ctrl9
#if defined(QMI8658_HANDSHAKE_NEW)
	#if defined(QMI8658_HANDSHAKE_TO_STATUS)
	unsigned char status_reg = Qmi8658Register_StatusInt;	
	unsigned char cmd_done = 0x80;
	#else
	unsigned char status_reg = Qmi8658Register_Status1;// Qmi8658Register_Status1
	unsigned char cmd_done = 0x01;
	#endif
	//count = 0;
	Qmi8658_read_reg(status_reg, &status1, 1);
	qmi8658_printf("status1=%d\n",status1);
	while(((status1&cmd_done)!=cmd_done)&&(count++<200))		// read statusINT until bit7 is 1
	{
		qmi8658_delay_ms(1);
		Qmi8658_read_reg(status_reg, &status1, 1);
		//qmi8658_printf("first %d status_reg: 0x%x\n", count,status1);
	}
	qmi8658_printf("Qmi8658_config_fifo ctrl9 done-1: %d\n", count);
	Qmi8658_write_reg(Qmi8658Register_Ctrl9, Qmi8658_Ctrl9_Cmd_NOP);	// write commond  0x00 to ctrl9
	count = 0;
	Qmi8658_read_reg(status_reg, &status1, 1);
	while(((status1&cmd_done)==cmd_done)&&(count++<200))		// read statusINT until bit7 is 0
	{
		qmi8658_delay_ms(1);
		qmi8658_printf("second %d status_reg: 0x%x\n", count,status1);
		Qmi8658_read_reg(status_reg, &status1, 1);
	}
	qmi8658_printf("ctrl9 done-2: %d\n", count);
#else
	while(((status1&QMI8658_STATUS1_CMD_DONE)==0)&&(count++<200))
	{
		Qmi8658_delay(2);
		Qmi8658_read_reg(Qmi8658Register_Status1, &status1, sizeof(status1));
	}
	//qmi8658_printf("fifo rst done : %d\n", count);
#endif

	//qmi8658_i2c_deinit(pi2c);

}

void Qmi8658_config_fifo(unsigned char watermark,enum Qmi8658_FifoSize size,enum Qmi8658_FifoMode mode,enum Qmi8658_fifo_format format)
{
	void* pi2c = qmi8658_i2c_init();
	Qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);
	#if 1
	Qmi8658_delay(1);
	#else
	WaitMs(1);
	#endif
	qmi8658_config.fifo_format = format;		//QMI8658_FORMAT_12_BYTES;
	qmi8658_config.fifo_ctrl = (unsigned char)size | (unsigned char)mode;

	Qmi8658_write_reg(Qmi8658Register_FifoCtrl, qmi8658_config.fifo_ctrl);
	Qmi8658_write_reg(Qmi8658Register_FifoWmkTh, (unsigned char)watermark);
	Qmi8658_send_ctl9cmd(Qmi8658_Ctrl9_Cmd_Rst_Fifo);

	if(format == QMI8658_FORMAT_ACCEL_6_BYTES)
		Qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x01);
	else if(format == QMI8658_FORMAT_GYRO_6_BYTES)
		Qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x02);
	else if(format == QMI8658_FORMAT_12_BYTES)
		Qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x03);
	qmi8658_i2c_deinit(pi2c);
}

unsigned short Qmi8658_read_fifo(unsigned char* data)
{
	unsigned char fifo_status[2] = {0,0};
	unsigned short fifo_bytes = 0;
	unsigned short fifo_level = 0;
	//unsigned short i;
	void* pi2c = qmi8658_i2c_init();
	Qmi8658_send_ctl9cmd(Qmi8658_Ctrl9_Cmd_Req_Fifo);

	Qmi8658_read_reg(Qmi8658Register_FifoCount, fifo_status, 2);
	fifo_bytes = (unsigned short)(((fifo_status[1]&0x03)<<8)|fifo_status[0]);
	fifo_bytes *= 2;
	if((qmi8658_config.fifo_format == QMI8658_FORMAT_ACCEL_6_BYTES)||(qmi8658_config.fifo_format == QMI8658_FORMAT_GYRO_6_BYTES))
	{
		fifo_level = fifo_bytes/6; // one sensor
		fifo_bytes = fifo_level*6;
	}
	else if(qmi8658_config.fifo_format == QMI8658_FORMAT_12_BYTES)
	{
		fifo_level = fifo_bytes/12; // two sensor
		fifo_bytes = fifo_level*12;
	}
	qmi8658_printf("fifo byte=%d level=%d\n", fifo_bytes, fifo_level);
	if(fifo_level > 0)
	{
		#if 0
		for(i=1; i<fifo_level; i++)
		{
			Qmi8658_read_reg(Qmi8658Register_FifoData, data+i*12, 12);
		}
		#else
		Qmi8658_read_reg(Qmi8658Register_FifoData, data, fifo_bytes);
		#endif
	}	
	Qmi8658_write_reg(Qmi8658Register_FifoCtrl, qmi8658_config.fifo_ctrl);

	qmi8658_i2c_deinit(pi2c);
	return fifo_level;
}


void Qmi8658_get_fifo_format(enum Qmi8658_fifo_format *format)
{
	if(format)
		*format = qmi8658_config.fifo_format;
}

#endif

unsigned char Qmi8658_readStatusInt(void)
{
	unsigned char status_int;
	void* pi2c = qmi8658_i2c_init();
	Qmi8658_read_reg(Qmi8658Register_StatusInt, &status_int, 1);
	//printf("status[0x%x	0x%x]\n",status[0],status[1]);
	qmi8658_i2c_deinit(pi2c);
	return status_int;
}

unsigned char Qmi8658_readStatus0(void)
{
	unsigned char status[2];
	void* pi2c = qmi8658_i2c_init();
	Qmi8658_read_reg(Qmi8658Register_Status0, status, 1);
	//printf("status[0x%x	0x%x]\n",status[0],status[1]);
	qmi8658_i2c_deinit(pi2c);
	return status[0];
}
/*!
 * \brief Blocking read of data status register 1 (::Qmi8658Register_Status1).
 * \returns Status byte \see STATUS1 for flag definitions.
 */
unsigned char Qmi8658_readStatus1(void)
{
	unsigned char status;
	void* pi2c = qmi8658_i2c_init();
	Qmi8658_read_reg(Qmi8658Register_Status1, &status, 1);
	qmi8658_i2c_deinit(pi2c);
	return status;
}

float Qmi8658_readTemp(void)
{
	unsigned char buf[2];
	short temp = 0;
	float temp_f = 0;

	void* pi2c = qmi8658_i2c_init();
	
	Qmi8658_read_reg(Qmi8658Register_Tempearture_L, buf, 2);
	temp = ((short)buf[1]<<8)|buf[0];
	temp_f = (float)temp/256.0f;
	qmi8658_i2c_deinit(pi2c);
	return temp_f;
}

void Qmi8658_read_acc_xyz(float acc_xyz[3])
{
	unsigned char	buf_reg[6];
	short 			raw_acc_xyz[3];
	int32  xyz[3];
	void* pi2c = qmi8658_i2c_init();
	
	Qmi8658_read_reg(Qmi8658Register_Ax_L, buf_reg, 6); 	// 0x19, 25
	raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1]<<8) |( buf_reg[0]));
	raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3]<<8) |( buf_reg[2]));
	raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5]<<8) |( buf_reg[4]));

	
	acc_xyz[0] = (raw_acc_xyz[0]*ONE_G)/acc_lsb_div;
	acc_xyz[1] = (raw_acc_xyz[1]*ONE_G)/acc_lsb_div;
	acc_xyz[2] = (raw_acc_xyz[2]*ONE_G)/acc_lsb_div;
	
	xyz[0]=(int32)((raw_acc_xyz[0]*1000)/acc_lsb_div);
	xyz[1]=(int32)((raw_acc_xyz[1]*1000)/acc_lsb_div);
	xyz[2]=(int32)((raw_acc_xyz[2]*1000)/acc_lsb_div);
	
	qmi8658_i2c_deinit(pi2c);
}



void Qmi8658_read_gyro_xyz(float gyro_xyz[3])
{
	unsigned char	buf_reg[6];
	short 			raw_gyro_xyz[3];
	void* pi2c = qmi8658_i2c_init();
	
	Qmi8658_read_reg(Qmi8658Register_Gx_L, buf_reg, 6);  	// 0x1f, 31
	raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[1]<<8) |( buf_reg[0]));
	raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[3]<<8) |( buf_reg[2]));
	raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[5]<<8) |( buf_reg[4]));	

	gyro_xyz[0] = (raw_gyro_xyz[0]*1.0f)/gyro_lsb_div;
	gyro_xyz[1] = (raw_gyro_xyz[1]*1.0f)/gyro_lsb_div;
	gyro_xyz[2] = (raw_gyro_xyz[2]*1.0f)/gyro_lsb_div;

	qmi8658_i2c_deinit(pi2c);
}

void Qmi8658_read_xyz(float acc[3], float gyro[3], unsigned int *tim_count)
{
	unsigned char	buf_reg[12];
	short 			raw_acc_xyz[3];
	short 			raw_gyro_xyz[3];
//	float acc_t[3];
//	float gyro_t[3];
	void* pi2c = qmi8658_i2c_init();
	if(tim_count)
	{
		unsigned char	buf[3];
		unsigned int timestamp;
		Qmi8658_read_reg(Qmi8658Register_Timestamp_L, buf, 3);	// 0x18	24
		timestamp = (unsigned int)(((unsigned int)buf[2]<<16)|((unsigned int)buf[1]<<8)|buf[0]);
		if(timestamp > imu_timestamp)
			imu_timestamp = timestamp;
		else
			imu_timestamp = (timestamp+0x1000000-imu_timestamp);

		*tim_count = imu_timestamp;		
	}

	Qmi8658_read_reg(Qmi8658Register_Ax_L, buf_reg, 12); 	// 0x19, 25
	raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1]<<8) |( buf_reg[0]));
	raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3]<<8) |( buf_reg[2]));
	raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5]<<8) |( buf_reg[4]));

	raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7]<<8) |( buf_reg[6]));
	raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9]<<8) |( buf_reg[8]));
	raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11]<<8) |( buf_reg[10]));

#if defined(QMI8658_UINT_MG_DPS)
	// mg
	acc[AXIS_X] = (float)(raw_acc_xyz[AXIS_X]*1000.0f)/acc_lsb_div;
	acc[AXIS_Y] = (float)(raw_acc_xyz[AXIS_Y]*1000.0f)/acc_lsb_div;
	acc[AXIS_Z] = (float)(raw_acc_xyz[AXIS_Z]*1000.0f)/acc_lsb_div;
#else
	// m/s2
	acc[AXIS_X] = (float)(raw_acc_xyz[AXIS_X]*ONE_G)/acc_lsb_div;
	acc[AXIS_Y] = (float)(raw_acc_xyz[AXIS_Y]*ONE_G)/acc_lsb_div;
	acc[AXIS_Z] = (float)(raw_acc_xyz[AXIS_Z]*ONE_G)/acc_lsb_div;
#endif
//	acc[AXIS_X] = imu_map.sign[AXIS_X]*acc_t[imu_map.map[AXIS_X]];
//	acc[AXIS_Y] = imu_map.sign[AXIS_Y]*acc_t[imu_map.map[AXIS_Y]];
//	acc[AXIS_Z] = imu_map.sign[AXIS_Z]*acc_t[imu_map.map[AXIS_Z]];

#if defined(QMI8658_UINT_MG_DPS)
	// dps
	gyro[0] = (float)(raw_gyro_xyz[0]*1.0f)/gyro_lsb_div;
	gyro[1] = (float)(raw_gyro_xyz[1]*1.0f)/gyro_lsb_div;
	gyro[2] = (float)(raw_gyro_xyz[2]*1.0f)/gyro_lsb_div;
#else
	// rad/s
	gyro[AXIS_X] = (float)(raw_gyro_xyz[AXIS_X]*0.01745f)/gyro_lsb_div;		// *pi/180
	gyro[AXIS_Y] = (float)(raw_gyro_xyz[AXIS_Y]*0.01745f)/gyro_lsb_div;
	gyro[AXIS_Z] = (float)(raw_gyro_xyz[AXIS_Z]*0.01745f)/gyro_lsb_div;
#endif	
//	gyro[AXIS_X] = imu_map.sign[AXIS_X]*gyro_t[imu_map.map[AXIS_X]];
//	gyro[AXIS_Y] = imu_map.sign[AXIS_Y]*gyro_t[imu_map.map[AXIS_Y]];
//	gyro[AXIS_Z] = imu_map.sign[AXIS_Z]*gyro_t[imu_map.map[AXIS_Z]];

	qmi8658_i2c_deinit(pi2c);
}


void Qmi8658_read_xyz_raw(short raw_acc_xyz[3], short raw_gyro_xyz[3], unsigned int *tim_count)
{
	unsigned char	buf_reg[12];
	void* pi2c = qmi8658_i2c_init();
	if(tim_count)
	{
		unsigned char	buf[3];
		unsigned int timestamp;
		Qmi8658_read_reg(Qmi8658Register_Timestamp_L, buf, 3);	// 0x18	24
		timestamp = (unsigned int)(((unsigned int)buf[2]<<16)|((unsigned int)buf[1]<<8)|buf[0]);
		if(timestamp > imu_timestamp)
			imu_timestamp = timestamp;
		else
			imu_timestamp = (timestamp+0x1000000-imu_timestamp);

		*tim_count = imu_timestamp;	
	}
	if(raw_acc_xyz && raw_gyro_xyz)
	{
		Qmi8658_read_reg(Qmi8658Register_Ax_L, buf_reg, 12); 	// 0x19, 25
		raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1]<<8) |( buf_reg[0]));
		raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3]<<8) |( buf_reg[2]));
		raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5]<<8) |( buf_reg[4]));
		raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7]<<8) |( buf_reg[6]));
		raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9]<<8) |( buf_reg[8]));
		raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11]<<8) |( buf_reg[10]));
	}
	else if(raw_acc_xyz)
	{
		Qmi8658_read_reg(Qmi8658Register_Ax_L, buf_reg, 6); 	// 0x19, 25
		raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1]<<8) |( buf_reg[0]));
		raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3]<<8) |( buf_reg[2]));
		raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5]<<8) |( buf_reg[4]));
	}
	else if(raw_gyro_xyz)
	{
		Qmi8658_read_reg(Qmi8658Register_Gx_L, buf_reg, 6); 	// 0x19, 25
		raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[1]<<8) |( buf_reg[0]));
		raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[3]<<8) |( buf_reg[2]));
		raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[5]<<8) |( buf_reg[4]));
	}

	qmi8658_i2c_deinit(pi2c);
}


void Qmi8658_read_ae(float quat[4], float velocity[3])
{
	unsigned char	buf_reg[14];
	short 			raw_q_xyz[4];
	short 			raw_v_xyz[3];
	void* pi2c = qmi8658_i2c_init();
	Qmi8658_read_reg(Qmi8658Register_Q1_L, buf_reg, 14);
	raw_q_xyz[0] = (short)((unsigned short)(buf_reg[1]<<8) |( buf_reg[0]));
	raw_q_xyz[1] = (short)((unsigned short)(buf_reg[3]<<8) |( buf_reg[2]));
	raw_q_xyz[2] = (short)((unsigned short)(buf_reg[5]<<8) |( buf_reg[4]));
	raw_q_xyz[3] = (short)((unsigned short)(buf_reg[7]<<8) |( buf_reg[6]));

	raw_v_xyz[1] = (short)((unsigned short)(buf_reg[9]<<8) |( buf_reg[8]));
	raw_v_xyz[2] = (short)((unsigned short)(buf_reg[11]<<8) |( buf_reg[10]));
	raw_v_xyz[2] = (short)((unsigned short)(buf_reg[13]<<8) |( buf_reg[12]));

	quat[0] = (float)(raw_q_xyz[0]*1.0f)/ae_q_lsb_div;
	quat[1] = (float)(raw_q_xyz[1]*1.0f)/ae_q_lsb_div;
	quat[2] = (float)(raw_q_xyz[2]*1.0f)/ae_q_lsb_div;
	quat[3] = (float)(raw_q_xyz[3]*1.0f)/ae_q_lsb_div;

	velocity[0] = (float)(raw_v_xyz[0]*1.0f)/ae_v_lsb_div;
	velocity[1] = (float)(raw_v_xyz[1]*1.0f)/ae_v_lsb_div;
	velocity[2] = (float)(raw_v_xyz[2]*1.0f)/ae_v_lsb_div;
	qmi8658_i2c_deinit(pi2c);
}

void Qmi8658_enableWakeOnMotion(enum Qmi8658_Interrupt int_set, enum Qmi8658_WakeOnMotionThreshold threshold, unsigned char blankingTime)
{
	unsigned char cal1_1_reg = (unsigned char)threshold;
	unsigned char cal1_2_reg  = (unsigned char)int_set | (blankingTime & 0x3F);
	unsigned char status1 = 0;
	int count = 0;
	void* pi2c = qmi8658_i2c_init();
	Qmi8658_write_reg(Qmi8658Register_Ctrl7, QMI8658_CTRL7_DISABLE_ALL);
	Qmi8658_config_acc(Qmi8658AccRange_8g, Qmi8658AccOdr_LowPower_21Hz, Qmi8658Lpf_Disable, Qmi8658St_Disable);

	Qmi8658_write_reg(Qmi8658Register_Cal1_L, cal1_1_reg);
	Qmi8658_write_reg(Qmi8658Register_Cal1_H, cal1_2_reg);
	// ctrl9 wom setting
	Qmi8658_write_reg(Qmi8658Register_Ctrl9, Qmi8658_Ctrl9_Cmd_WoM_Setting);
	while(((status1&QMI8658_STATUS1_CMD_DONE)==0)&&(count++<200))
	{
		Qmi8658_delay(2);
		Qmi8658_read_reg(Qmi8658Register_Status1, &status1, sizeof(status1));
	}
	// ctrl9 wom setting
	Qmi8658_write_reg(Qmi8658Register_Ctrl7, QMI8658_CTRL7_ACC_ENABLE);
	qmi8658_i2c_deinit(pi2c);
}

void Qmi8658_disableWakeOnMotion(void)
{
	unsigned char status1 = 0;
	int count = 0;
	void* pi2c = qmi8658_i2c_init();
	Qmi8658_write_reg(Qmi8658Register_Ctrl7, QMI8658_CTRL7_DISABLE_ALL);
	Qmi8658_write_reg(Qmi8658Register_Cal1_L, 0);
	Qmi8658_write_reg(Qmi8658Register_Cal1_H, 0);
	
	Qmi8658_write_reg(Qmi8658Register_Ctrl9, Qmi8658_Ctrl9_Cmd_WoM_Setting);
	while(((status1&QMI8658_STATUS1_CMD_DONE)==0)&&(count++<200))
	{
		Qmi8658_delay(2);
		Qmi8658_read_reg(Qmi8658Register_Status1, &status1, sizeof(status1));
	}
	qmi8658_i2c_deinit(pi2c);
}

void Qmi8658_enableSensors(unsigned char enableFlags)
{
	void* pi2c = qmi8658_i2c_init();
	if(enableFlags & QMI8658_CONFIG_AE_ENABLE)
	{
		enableFlags |= QMI8658_CTRL7_ACC_ENABLE | QMI8658_CTRL7_GYR_ENABLE;
	}
#if defined(QMI8658_SYNC_SAMPLE_MODE)
	Qmi8658_write_reg(Qmi8658Register_Ctrl7, enableFlags | 0x80);
#else
	Qmi8658_write_reg(Qmi8658Register_Ctrl7, enableFlags & QMI8658_CTRL7_ENABLE_MASK);
#endif

	qmi8658_i2c_deinit(pi2c);
}

void Qmi8658_Config_apply(struct Qmi8658Config const* config)
{
	unsigned char fisSensors = config->inputSelection;
	void* pi2c = qmi8658_i2c_init();
	if(fisSensors & QMI8658_CONFIG_AE_ENABLE)
	{
		Qmi8658_config_ae(config->aeOdr);
	}
	else
	{
		if (config->inputSelection & QMI8658_CONFIG_ACC_ENABLE)
		{
			Qmi8658_config_acc(config->accRange, config->accOdr, Qmi8658Lpf_Disable, Qmi8658St_Disable);
		}
		if (config->inputSelection & QMI8658_CONFIG_GYR_ENABLE)
		{
			Qmi8658_config_gyro(config->gyrRange, config->gyrOdr, Qmi8658Lpf_Enable, Qmi8658St_Disable);
		}
	}

	if(config->inputSelection & QMI8658_CONFIG_MAG_ENABLE)
	{
		Qmi8658_config_mag(config->magDev, config->magOdr);
	}
#if defined(QMI8658_USE_FIFO)
	//Qmi8658_config_fifo(Qmi8658_Fifo_WmkHalf,Qmi8658_Fifo_64,Qmi8658_Fifo_Stream,QMI8658_FORMAT_12_BYTES);
#endif
	
	Qmi8658_enableSensors(fisSensors);	


	qmi8658_i2c_deinit(pi2c);
}



unsigned char stepConfig(unsigned short odr)  //11hz=0x0e��
{
	unsigned short ped_fix_peak,ped_sample_cnt,ped_time_up,ped_fix_peak2peak;
	unsigned char ped_time_low,ped_time_cnt_entry,ped_fix_precision,ped_sig_count,basedODR;
	void* pi2c = qmi8658_i2c_init();
	
	float finalRate;
	finalRate = 200.0/odr;  //14.285
	ped_sample_cnt = (unsigned short)(0x0032 / finalRate) ;//6;//(unsigned short)(0x0032 / finalRate) ;
	ped_fix_peak2peak = 0x00CC;
	ped_fix_peak = 0x00CC;
	ped_time_up = (unsigned short)(200.0 / finalRate);
	ped_time_low= (unsigned char) (20 / finalRate) ;
	ped_time_cnt_entry = 1;
	ped_fix_precision = 0;
	ped_sig_count = 1;//�Ʋ�����1
	

	Qmi8658_write_reg(Qmi8658Register_Cal1_L, ped_sample_cnt & 0xFF);
	Qmi8658_write_reg(Qmi8658Register_Cal1_H, (ped_sample_cnt >> 8) & 0xFF);
	Qmi8658_write_reg(Qmi8658Register_Cal2_L, ped_fix_peak2peak & 0xFF);
	Qmi8658_write_reg(Qmi8658Register_Cal2_H, (ped_fix_peak2peak >> 8) & 0xFF);
	Qmi8658_write_reg(Qmi8658Register_Cal3_L, ped_fix_peak & 0xFF);
	Qmi8658_write_reg(Qmi8658Register_Cal3_H, (ped_fix_peak >> 8) & 0xFF);
	Qmi8658_write_reg(Qmi8658Register_Cal4_L, 0x01);
	qmi8658_printf("+");

	
	Qmi8658_send_ctl9cmd (Qmi8658_Ctrl9_Cmd_EnablePedometer);
	qmi8658_printf(".");
	///////////////////////////////
	Qmi8658_write_reg(Qmi8658Register_Cal1_L,ped_time_up & 0xFF);
	Qmi8658_write_reg(Qmi8658Register_Cal1_H, (ped_time_up >> 8) & 0xFF);
	Qmi8658_write_reg(Qmi8658Register_Cal2_L,ped_time_low );
	Qmi8658_write_reg(Qmi8658Register_Cal2_H, ped_time_cnt_entry );
	Qmi8658_write_reg(Qmi8658Register_Cal3_L, ped_fix_precision );
	Qmi8658_write_reg(Qmi8658Register_Cal3_H, ped_sig_count );
	Qmi8658_write_reg(Qmi8658Register_Cal4_L, 0x02 );
	qmi8658_printf("-");
	Qmi8658_send_ctl9cmd (Qmi8658_Ctrl9_Cmd_EnablePedometer);
	qmi8658_printf("*");
	qmi8658_i2c_deinit(pi2c);


}
unsigned char Qmi8658_init(void)
{
	unsigned char qmi8658_chip_id = 0x00;
	unsigned char qmi8658_revision_id = 0x00;
	unsigned char qmi8658_slave[2] = {QMI8658_SLAVE_ADDR_L, QMI8658_SLAVE_ADDR_H};
	unsigned char iCount = 0;
	unsigned char Qmi8658Register_Ctrl8_value=0;
	void* pi2c = qmi8658_i2c_init();
	while((qmi8658_chip_id == 0x00)&&(iCount<2))
	{
		qmi8658_slave_addr = qmi8658_slave[iCount];
		Qmi8658_read_reg(Qmi8658Register_WhoAmI, &qmi8658_chip_id, 1);
		//qmi8658_printf("Qmi8658 ChipID-1=0x%02x \n", qmi8658_chip_id);
		if(qmi8658_chip_id == 0x05)
			break;
		iCount++;
	}
	//LOG("CHIP ID %d\n",qmi8658_chip_id);
	if(qmi8658_chip_id == 0x05)
	{
		unsigned char firmware_id[3];
		
		Qmi8658_read_reg(Qmi8658Register_Ctrl1, &qmi8658_revision_id, 1);
		qmi8658_printf("Qmi8658Register_Ctr1-1=0x%02x \n", qmi8658_revision_id);
		Qmi8658_write_reg(Qmi8658Register_Ctrl1, 0x60);
		
		Qmi8658_read_reg(Qmi8658Register_Ctrl1, &qmi8658_revision_id, 1);
		qmi8658_printf("Qmi8658Register_Ctr1-2=0x%02x \n", qmi8658_revision_id);

		qmi8658_chip_id = 0;
		Qmi8658_read_reg(Qmi8658Register_WhoAmI, &qmi8658_chip_id, 1);
		qmi8658_printf("Qmi8658 ChipID=0x%02x \n", qmi8658_chip_id);
		
		Qmi8658_read_reg(Qmi8658Register_Revision, &qmi8658_revision_id, 1);
		qmi8658_printf("Qmi8658_init slave=0x%x  Qmi8658Register_WhoAmI=0x%x 0x%x\n", qmi8658_slave_addr,qmi8658_chip_id,qmi8658_revision_id);
		Qmi8658_read_reg(0x49, firmware_id, 3);
		qmi8658_printf("Old Firmware ID[0x%x 0x%x 0x%x]\n", firmware_id[0],firmware_id[1],firmware_id[2]);

		qmi8658_config.inputSelection = QMI8658_CONFIG_ACC_ENABLE;//QMI8658_CONFIG_ACCGYR_ENABLE;
		
		qmi8658_config.accRange = Qmi8658AccRange_8g;
		qmi8658_config.accOdr = Qmi8658AccOdr_LowPower_11Hz;
		
		
		qmi8658_config.gyrRange = Qmi8658GyrRange_1024dps;		//Qmi8658GyrRange_2048dps   Qmi8658GyrRange_1024dps
		qmi8658_config.gyrOdr = Qmi8658GyrOdr_500Hz;
		qmi8658_config.magOdr = Qmi8658MagOdr_125Hz;
		qmi8658_config.magDev = MagDev_AKM09918;
		qmi8658_config.aeOdr = Qmi8658AeOdr_128Hz;
		
		Qmi8658_write_reg(Qmi8658Register_Ctrl7, 0x00);		//
		Qmi8658_write_reg(Qmi8658Register_Ctrl8, 0xD0);
		
		qmi8658_printf("1\n");
		stepConfig(qmi8658_config.accOdr);
		qmi8658_printf("2\n");
		Qmi8658_Config_apply(&qmi8658_config);
		
		
		if(0)
		{
			unsigned char read_data = 0x00;
				 pi2c = qmi8658_i2c_init();
				
			Qmi8658_read_reg(Qmi8658Register_Ctrl1, &read_data, 1);
			qmi8658_printf("Qmi8658Register_Ctrl1=0x%x \n", read_data);
			Qmi8658_read_reg(Qmi8658Register_Ctrl2, &read_data, 1);
			qmi8658_printf("Qmi8658Register_Ctrl2=0x%x \n", read_data);
			Qmi8658_read_reg(Qmi8658Register_Ctrl3, &read_data, 1);
			qmi8658_printf("Qmi8658Register_Ctrl3=0x%x \n", read_data);
			Qmi8658_read_reg(Qmi8658Register_Ctrl4, &read_data, 1);
			qmi8658_printf("Qmi8658Register_Ctrl4=0x%x \n", read_data);
			Qmi8658_read_reg(Qmi8658Register_Ctrl5, &read_data, 1);
			qmi8658_printf("Qmi8658Register_Ctrl5=0x%x \n", read_data);
			Qmi8658_read_reg(Qmi8658Register_Ctrl6, &read_data, 1);
			qmi8658_printf("Qmi8658Register_Ctrl6=0x%x \n", read_data);
			Qmi8658_read_reg(Qmi8658Register_Ctrl7, &read_data, 1);
			qmi8658_printf("Qmi8658Register_Ctrl7=0x%x \n", read_data);
				qmi8658_i2c_deinit(pi2c);
		}
//		Qmi8658_set_layout(2);
		qmi8658_i2c_deinit(pi2c);
		return 1;

	}
	else
	{
		qmi8658_printf("Qmi8658_init fail\n");
		qmi8658_chip_id = 0;
		qmi8658_i2c_deinit(pi2c);
		return 0;
	}
	//return qmi8658_chip_id;
}


uint8_t Qmi8658_get_chipid(uint8_t * chip_id)
{
	uint8_t  ret = 0;
	void* pi2c = qmi8658_i2c_init();	
	
	Qmi8658_read_reg(Qmi8658Register_WhoAmI, chip_id, 1);	
	if((*chip_id >= 0x05) && (*chip_id <= 0x06))
	{
		ret = 1;
	}	
	qmi8658_i2c_deinit(pi2c);
	
	return ret;
}

uint32_t  Qmi8658_step_read_stepcounter(uint32_t * step)
{
	uint8_t reg_data[3]={0};
	uint8_t	reg_47=0;
	uint32_t rslt;
	void* pi2c = qmi8658_i2c_init();

	#if 1
	Qmi8658_read_reg(Qmi8658Register_AccEl_X,reg_data,3);
	#else
	Qmi8658_read_reg(Qmi8658Register_AccEl_X,reg_data,1);
	Qmi8658_read_reg(Qmi8658Register_AccEl_Y,&reg_data[1],1);
	Qmi8658_read_reg(Qmi8658Register_AccEl_Z,&reg_data[2],1);
	#endif
	//Qmi8658_read_reg(47,&reg_47,1);
	//Qmi8658_read_reg(Qmi8658Register_AccEl_X,reg_data,3);
	
  rslt =reg_data[2]<< 16 |reg_data[1]<< 8 |reg_data[0];
	*step = 	rslt;
	//qmi8658_printf("reg_47=%d\n",reg_47);
	qmi8658_i2c_deinit(pi2c);
	return rslt;
}

uint8_t Qmi8658_standby(void)
{
	uint8_t chip_id;
	uint8_t tEmp = 0;	
	void* pi2c;
	
	if(!Qmi8658_get_chipid(&chip_id))
	{
		return 0;
	}
	else
	{
		void* pi2c = qmi8658_i2c_init();	
		//Qmi8658_write_reg(Qmi8658Register_Ctrl1,  0x61);
		Qmi8658_write_reg(Qmi8658Register_Ctrl8,  0xC0);
		Qmi8658_write_reg(Qmi8658Register_Ctrl7,  0x00);
		qmi8658_i2c_deinit(pi2c);		
		return 1;
	}	
}
#if 0
void Qmi8658_step_reset_stepcounter(void)
{
	uint8_t tEmp;
	uint8_t reg13;
	void* pi2c = qmi8658_i2c_init();
	//WaitMs(1);
	Qmi8658_read_reg(0x13, &reg13, 1);
	tEmp = 0x80;
	Qmi8658_write_reg(0x13, &tEmp, 1);		// clear step
	tEmp = reg13;
	Qmi8658_write_reg(0x13, &tEmp, 1);		// 

	qmi8658_i2c_deinit(pi2c);
	
}
#endif


void qmi8658_step_test_demo(void)
{
	uint32_t tEmp;
	static uint8_t qmistatus = 0;
	if(0 == qmistatus)
	{
		if(Qmi8658_init())
		{
			qmistatus = 1;
			#ifdef PRINT_LOG
			qmi8658_printf("QMI8658 is initialized!\r\n");
			#endif
		}
	}
	else
	{
		
		Qmi8658_step_read_stepcounter(&tEmp);
		//qmi8658_printf("step counter:\treset\r\n", tEmp);
	}
}

void qmi8658sleep(void)
{
		Qmi8658_standby();

}








