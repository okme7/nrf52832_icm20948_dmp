/*----------------------------------------------------------------------------------------------------
** Created by        : [����ķ]
** Created date      : 2018-12-24
** Version           : 1.0
** Descriptions      : W25Q128 SPI Flash��������
**---------------------------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_esb.h"
#include "nrf_esb_error_codes.h"
#include "nrf_drv_spi.h"
#include "w25q128.h"

//SPI��������ʵ��ID,ID�������Ŷ�Ӧ��0:SPI0  1:SPI1 2:SPI2
#define SPI_INSTANCE  0 
//��������Ϊspi��SPI��������ʵ��
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  
//SPI������ɱ�־
static volatile bool spi_xfer_done;  
//SPI���ͻ������飬ʹ��EasyDMAʱһ��Ҫ����Ϊstatic����
static uint8_t    spi_tx_buf[6];  
//SPI���ջ������飬ʹ��EasyDMAʱһ��Ҫ����Ϊstatic����
static uint8_t    spi_rx_buf[6];  

//SPI���ͻ������飬ʹ��EasyDMAʱһ��Ҫ����Ϊstatic����
static uint8_t spi_dmp_tx_buf[256];  
//SPI���ջ������飬ʹ��EasyDMAʱһ��Ҫ����Ϊstatic����
static uint8_t spi_dmp_rx_buf[256];  

static float gyro_scale_factor;
static float accel_scale_factor;


/*****************************************************************************
** ��  ����д��һ���ֽ�
** ��  ����Dat����д�������
** ����ֵ����
******************************************************************************/
void Spi_WriteOneByte(uint8_t Dat)
{   
	  spi_tx_buf[0] = Dat;
	  spi_xfer_done = false;
	  SPIFlash_CS_LOW;
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 1, spi_rx_buf, 0));
    while(!spi_xfer_done);
	  SPIFlash_CS_HIGH;
}

void ICM94_AUX_ReadRegs( uint8_t slaveAddr, uint8_t readAddr, uint8_t *readData, uint8_t lens )
{
	
}










/*****************************************************************************
** ��  ����дʹ��
** ��  ������
** ����ֵ����
******************************************************************************/
static void SpiFlash_Write_Enable(void)
{
  //Spi_WriteOneByte(SPIFlash_WriteEnable);
	;
}

/*****************************************************************************
** ��  ������ȡicm20948
** ��  ������
** ����ֵ��
******************************************************************************/


uint8_t SpiICM20948_Read(void)
{
  
	//׼������
	spi_tx_buf[0] = (0x80|WHO_AM_I_s);
	spi_tx_buf[1] = 0x00;
	/*spi_tx_buf[2] = 0x00;
	spi_tx_buf[3] = 0x00;
	spi_tx_buf[4] = 0xFF;
	spi_tx_buf[5] = 0xFF;*/
	//������ɱ�־����Ϊfalse
	spi_xfer_done = false;
	//����CS��ʹ��W25Q128FV
	SPIFlash_CS_LOW;
	//�������ݴ���
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 2, spi_rx_buf, 2));
	//�ȴ�SPI�������
  while(!spi_xfer_done);
  //����CS���ͷ�W25Q128FV
	SPIFlash_CS_HIGH;
	//����������������ֽڲ��Ƕ�ȡ��ID
	//printf("who am I 0X%x \n",spi_rx_buf[1]);
	return spi_rx_buf[1];
}

static void select_user_bank(userbank ub)
{
	spi_tx_buf[0] = 0x00 | 0x7F;
	spi_tx_buf[1] = ub;
	spi_xfer_done = false;
	
	SPIFlash_CS_LOW;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 2, spi_rx_buf, 0));
	while(!spi_xfer_done);
	SPIFlash_CS_HIGH;
}

uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg)
{
	
	
	select_user_bank(ub);
	spi_tx_buf[0] = READ | reg;
	spi_tx_buf[1] = 0x00;
	spi_xfer_done = false;
	SPIFlash_CS_LOW;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 2, spi_rx_buf, 2));
	while(!spi_xfer_done);
	SPIFlash_CS_HIGH;
	//printf("0x%x recdata 0x%x 0X%x \r\n",reg,spi_rx_buf[0],spi_rx_buf[1]);
	return spi_rx_buf[1];
}

void  write_single_icm20948_reg(userbank ub,uint8_t writeAddr, uint8_t writeData)
{
	//return myiic_write_reg(ICM20602_ADDRESS,reg,val);
	
	select_user_bank(ub);
	spi_xfer_done = false;
	spi_tx_buf[0] =  WRITE | writeAddr;
	spi_tx_buf[1] = writeData;	
	SPIFlash_CS_LOW;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 2, spi_rx_buf, 0));
	
	//�ȴ�SPI�������
  while(!spi_xfer_done);
	SPIFlash_CS_HIGH;
	//printf("write:0x%x data 0X%x\r\n",spi_tx_buf[0],spi_tx_buf[1]);
}

static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len)
{
	uint8_t write_reg = WRITE | reg;
	select_user_bank(ub);
	spi_xfer_done = false;
	SPIFlash_CS_LOW;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &write_reg, 1, spi_rx_buf, 0));
	//�ȴ�SPI�������
  while(!spi_xfer_done);
	
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, val, len, spi_rx_buf, 0));
	//�ȴ�SPI�������
  while(!spi_xfer_done);

	SPIFlash_CS_HIGH;
}


	
static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len)
{
	uint16_t i;
	uint8_t read_reg = READ | reg;
	static uint8_t reg_val[6];
	for(i = 0;i <len;i++)
	{
		spi_tx_buf[i] = 0xff;
	}
	
	select_user_bank(ub);
	spi_xfer_done = false;
	SPIFlash_CS_LOW;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &read_reg, 1, spi_rx_buf, 0));
	//�ȴ�SPI�������
  while(!spi_xfer_done);
	
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, len, reg_val, len));
	//�ȴ�SPI�������
  while(!spi_xfer_done);

	SPIFlash_CS_HIGH;

	return reg_val;
}


void write_multiple_spi(uint8_t reg, uint8_t *data, uint32_t len)
{
	uint8_t write_reg = WRITE | reg;
	spi_xfer_done = false;
	SPIFlash_CS_LOW;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &write_reg, 1, spi_dmp_rx_buf, 0));
	//�ȴ�SPI�������
  while(!spi_xfer_done);
	
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, data, len, spi_dmp_rx_buf, 0));
	//�ȴ�SPI�������
  while(!spi_xfer_done);

	SPIFlash_CS_HIGH;
}

void read_multiple_spi(uint8_t reg, uint8_t *buff, uint32_t len)
{
	uint16_t i;
	uint8_t read_reg = READ | reg;
	for(i = 0;i <len;i++)
	{
		spi_dmp_tx_buf[i] = 0x00;
	}
	
	spi_xfer_done = false;
	SPIFlash_CS_LOW;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &read_reg, 1, spi_dmp_rx_buf, 0));
	//�ȴ�SPI�������
  while(!spi_xfer_done);
	
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_dmp_tx_buf, 0, buff, len));
	//�ȴ�SPI�������
  while(!spi_xfer_done);

	SPIFlash_CS_HIGH;
}
//
void MPU_Get_Gyroscope(short* gx, short* gy, short* gz)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_GYRO_XOUT_H, 6);
	*gx = (int16_t)(temp[0] << 8 | temp[1]);
	*gy = (int16_t)(temp[2] << 8 | temp[3]);
	*gz = (int16_t)(temp[4] << 8 | temp[5]);
}

void MPU_Get_Accelerometer(short* ax, short* ay, short* az)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);

	*ax = (int16_t)(temp[0] << 8 | temp[1]);
	*ay = (int16_t)(temp[2] << 8 | temp[3]);
	*az = (int16_t)(temp[4] << 8 | temp[5]);
}

void icm20948_i2c_init()
{
	write_single_icm20948_reg(ub_0, B0_INT_PIN_CFG, 0x30);   //0x10  ��bit5��1 - INT1���ŵĵ�ƽ����ֱ���ж�״̬�������0 - INT1����ָʾ�ж����������Ϊ50΢�롣��
	
	write_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL, 0x4d);    //400khz

	write_single_icm20948_reg(ub_3, B3_I2C_MST_DELAY_CTRL, 0x01);
	
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);
}

static void i2c_mag_wirte(uint8_t reg, uint8_t vaule)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, AK9916_Addr);
	nrf_delay_us(200);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	nrf_delay_us(200);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_DO, vaule);
	nrf_delay_us(800);
}

static uint8_t i2c_mag_read(uint8_t reg)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, AK9916_Addr|0x80);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_DO, 0xff);
//	nrf_delay_us(800);
		nrf_delay_ms(1);
	return read_single_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00);
}

static uint8_t* i2c_read_multiple_reg(uint8_t reg, uint8_t len)
{
		uint16_t i;
		static uint8_t reg_val[8];
		write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, AK9916_Addr|0x80);
		write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x80|len);
		write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
		nrf_delay_ms(1);
		for(i = 0;i < len; i++)
		{
			reg_val[i] = read_single_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00 + i);
		}
		return reg_val;
}
	
void i2c_mag_init()
{
	i2c_mag_wirte(AK9916_CNTL3, 0x01);
	i2c_mag_wirte(AK9916_CNTL2, 0x08);
}


void MPU_Get_Magnetism(short* mx, short* my, short* mz)
{
		uint8_t* temp;
		temp = i2c_read_multiple_reg(AK9916_HXL, 8);
		*mx = (int16_t)(temp[1] << 8 | temp[0]);
		*my = (int16_t)(temp[3] << 8 | temp[2]);
		*mz = (int16_t)(temp[5] << 8 | temp[4]);
}

//

void icm20948_device_reset()
{
	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, 0x80 | 0x41);
	nrf_delay_ms(100);
}

void icm20948_wakeup()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val &= 0xBF;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	nrf_delay_ms(100);
}

void icm20948_spi_slave_enable()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x30;        //0x10  0x30
	
	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_clock_source(uint8_t source)
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= source;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
}

void icm20948_odr_align_enable()
{
	write_single_icm20948_reg(ub_2, B2_ODR_ALIGN_EN, 0x01);
}

void icm20948_gyro_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	new_val |= config << 3;

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
	new_val |= config << 3;
	write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);
}

void icm20948_gyro_sample_rate_divider(uint8_t divider)
{
	write_single_icm20948_reg(ub_2, B2_GYRO_SMPLRT_DIV, divider);
}

void icm20948_accel_sample_rate_divider(uint16_t divider)
{
	uint8_t divider_1 = (uint8_t)(divider >> 8);
	uint8_t divider_2 = (uint8_t)(0x0F & divider);

	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_1, divider_1);
	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_2, divider_2);
}
void icm20948_gyro_read(axises* data)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_GYRO_XOUT_H, 6);
	//printf("gyro 0x%x%x 0x%x%x 0x%x%x \r\n",temp[0],temp[1],temp[2],temp[3],temp[4],temp[5]);
	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]);
	//printf("gyro x:%f y:%f z:%f\r\n",data->x,data->y,data->z);
}

void icm20948_accel_read(axises* data)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);
//	printf("accel 0x%x 0x%x 0x%x\r\n",(temp[0] << 8 | temp[1]),(temp[2] << 8 | temp[3]),(temp[4] << 8 | temp[5]));
	data->x = (int16_t)((temp[0] << 8) | temp[1]);
	data->y = (int16_t)((temp[2] << 8) | temp[3]);
	data->z = (int16_t)((temp[4] << 8) | temp[5]);

	//data->z = (int16_t)(temp[4] << 8 | temp[5]) + accel_scale_factor; 
	// Add scale factor because calibraiton function offset gravity acceleration.
	//esb_data_send(temp, 6);
//	printf("accel x:%f y:%f z:%f\r\n",data->x, data->y, data->z);
}
void icm20948_gyro_calibration()
{
	axises temp;
	int32_t gyro_bias[3] = {0};
	uint8_t gyro_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		icm20948_gyro_read(&temp);
		gyro_bias[0] += temp.x;
		gyro_bias[1] += temp.y;
		gyro_bias[2] += temp.z;
	}

	gyro_bias[0] /= 100;
	gyro_bias[1] /= 100;
	gyro_bias[2] /= 100;

	// Construct the gyro biases for push to the hardware gyro bias registers,
	// which are reset to zero upon device startup.
	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
	// Biases are additive, so change sign on calculated average gyro biases
	gyro_offset[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; 
	gyro_offset[1] = (-gyro_bias[0] / 4)       & 0xFF; 
	gyro_offset[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	gyro_offset[3] = (-gyro_bias[1] / 4)       & 0xFF;
	gyro_offset[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	gyro_offset[5] = (-gyro_bias[2] / 4)       & 0xFF;
	
	write_multiple_icm20948_reg(ub_2, B2_XG_OFFS_USRH, gyro_offset, 6);
}

void icm20948_accel_calibration()
{
	axises temp;
	uint8_t* temp2;
	uint8_t* temp3;
	uint8_t* temp4;
	
	int32_t accel_bias[3] = {0};
	int32_t accel_bias_reg[3] = {0};
	uint8_t accel_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		icm20948_accel_read(&temp);
		accel_bias[0] += temp.x;
		accel_bias[1] += temp.y;
		accel_bias[2] += temp.z;
	}

	accel_bias[0] /= 100;
	accel_bias[1] /= 100;
	accel_bias[2] /= 100;

	uint8_t mask_bit[3] = {0, 0, 0};

	temp2 = read_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, 2);
	accel_bias_reg[0] = (int32_t)(temp2[0] << 8 | temp2[1]);
	mask_bit[0] = temp2[1] & 0x01;

	temp3 = read_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, 2);
	accel_bias_reg[1] = (int32_t)(temp3[0] << 8 | temp3[1]);
	mask_bit[1] = temp3[1] & 0x01;

	temp4 = read_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, 2);
	accel_bias_reg[2] = (int32_t)(temp4[0] << 8 | temp4[1]);
	mask_bit[2] = temp4[1] & 0x01;

	accel_bias_reg[0] -= (accel_bias[0] / 8);
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  	accel_offset[1] = (accel_bias_reg[0])      & 0xFE;
	accel_offset[1] = accel_offset[1] | mask_bit[0];

	accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  	accel_offset[3] = (accel_bias_reg[1])      & 0xFE;
	accel_offset[3] = accel_offset[3] | mask_bit[1];

	accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	accel_offset[5] = (accel_bias_reg[2])      & 0xFE;
	accel_offset[5] = accel_offset[5] | mask_bit[2];
	
	write_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, &accel_offset[0], 2);
	write_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, &accel_offset[2], 2);
	write_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, &accel_offset[4], 2);
}

void icm20948_gyro_full_scale_select(gyro_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	
	switch(full_scale)
	{
		case _250dps :
			new_val |= 0x00;
			gyro_scale_factor = 131.0;
			break;
		case _500dps :
			new_val |= 0x02;
			gyro_scale_factor = 65.5;
			break;
		case _1000dps :
			new_val |= 0x04;
			gyro_scale_factor = 32.8;
			break;
		case _2000dps :
			new_val |= 0x06;
			gyro_scale_factor = 16.4;
			break;
	}

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_full_scale_select(accel_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
	
	switch(full_scale)
	{
		case _2g :
			new_val |= 0x00;
			accel_scale_factor = 16384;
			break;
		case _4g :
			new_val |= 0x02;
			accel_scale_factor = 8192;
			break;
		case _8g :
			new_val |= 0x04;
			accel_scale_factor = 4096;
			break;
		case _16g :
			new_val |= 0x06;
			accel_scale_factor = 2048;
			break;
	}

	write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);
}





void icm20948_gyro_read_dps(axises* data)
{
	icm20948_gyro_read(data);

	data->x /= gyro_scale_factor;
	data->y /= gyro_scale_factor;
	data->z /= gyro_scale_factor;
	
	//printf("gyro x:%f y:%f z:%f\r\n",data->x,data->y,data->z);
}

void icm20948_accel_read_g(axises* data)
{
	icm20948_accel_read(data);

	data->x /= accel_scale_factor;
	data->y /= accel_scale_factor;
	data->z /= accel_scale_factor;
	//printf("accel x:%f y:%f z:%f\r\n",data->x,data->y,data->z);
}



void icm20948_init(void)
{
	icm20948_device_reset(); //��λ������
	
	icm20948_wakeup();       //����
	
	icm20948_clock_source(1);    //ʱ��Դѡ��
	icm20948_odr_align_enable(); //����ʱ�����
	
	icm20948_spi_slave_enable();   //ʹ��spi�ӻ���i2c����
	
//	icm20948_gyro_low_pass_filter(7);  //��ͨ�˲���
//	icm20948_accel_low_pass_filter(7);

	icm20948_gyro_sample_rate_divider(0);   //��Ƶ  1.125khz
	icm20948_accel_sample_rate_divider(0);  //1.125khz
	
	icm20948_i2c_init();
	i2c_mag_init();
//	icm20948_gyro_calibration();
//	icm20948_accel_calibration();

	icm20948_gyro_full_scale_select(_2000dps);   //������Χ
	icm20948_accel_full_scale_select(_2g);
	
}





void SpiICM20948_Read_x(void)
{
//  uint16_t dat = 0;
	//׼������
	spi_tx_buf[0] = (0xad);
	spi_tx_buf[1] = 0xff;	
	spi_tx_buf[2] = 0xff;
	spi_tx_buf[3] = 0xff;
	spi_tx_buf[4] = 0xff;
	spi_tx_buf[5] = 0xf;
	//������ɱ�־����Ϊfalse
	spi_xfer_done = false;

	
	SPIFlash_CS_LOW;
	//�������ݴ���
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 6, spi_rx_buf, 6));
	//�ȴ�SPI�������
  while(!spi_xfer_done);
  //����CS���ͷ�W25Q128FV
	SPIFlash_CS_HIGH;
	//����������������ֽڲ��Ƕ�ȡ��ID
	//printf("%d %d %d %d %d %d\r\n",spi_rx_buf[0],spi_rx_buf[1],spi_rx_buf[2],spi_rx_buf[3],spi_rx_buf[4],spi_rx_buf[5]);
}
/*****************************************************************************
** ��  ������ȡW25Q128оƬID
** ��  ������
** ����ֵ��16λID��W25Q128оƬIDΪ��0xEF17
******************************************************************************/
uint16_t SpiFlash_ReadID(void)
{
  uint16_t dat = 0;
	//׼������
	//spi_tx_buf[0] = SPIFlash_ReadID;
	spi_tx_buf[1] = 0x00;
	spi_tx_buf[2] = 0x00;
	spi_tx_buf[3] = 0x00;
	spi_tx_buf[4] = 0xFF;
	spi_tx_buf[5] = 0xFF;
	//������ɱ�־����Ϊfalse
	spi_xfer_done = false;
	//����CS��ʹ��W25Q128FV
	SPIFlash_CS_LOW;
	//�������ݴ���
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 6, spi_rx_buf, 6));
	//�ȴ�SPI�������
  while(!spi_xfer_done);
  //����CS���ͷ�W25Q128FV
	SPIFlash_CS_HIGH;
	//����������������ֽڲ��Ƕ�ȡ��ID
	dat|=spi_rx_buf[4]<<8;  
	dat|=spi_rx_buf[5];	

  return dat;		
}
/*****************************************************************************
** ��  ������ȡW25Q128״̬�Ĵ���
** ��  ������
** ����ֵ��
******************************************************************************/
static uint8_t SpiFlash_ReadSR(void)
{
	  //spi_tx_buf[0] = SPIFlash_ReadStatusReg;
	  spi_tx_buf[1] = 0x00;
    //������ɱ�־����Ϊfalse
	  spi_xfer_done = false;
	  SPIFlash_CS_LOW;
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 2, spi_rx_buf, 2));
    while(!spi_xfer_done);
    SPIFlash_CS_HIGH;
    return spi_rx_buf[1];
}
//�ȴ�W25Q128����
void SpiFlash_Wait_Busy(void)   
{   
	while((SpiFlash_ReadSR()&0x01)==0x01);  		// �ȴ�BUSYλ���
} 
/*****************************************************************************
** ��  ��������������W25Q128FVSIG��С�Ĳ�����λ������
** ��  ����[in]SecAddr��������ַ
** ����ֵ����
******************************************************************************/
void SPIFlash_Erase_Sector(uint32_t SecAddr)
{
		//����дʹ������
    SpiFlash_Write_Enable();
		
		//������������
		//spi_tx_buf[0] = SPIFlash_SecErase;		
		//24λ��ַ
    spi_tx_buf[1] = (uint8_t)((SecAddr&0x00ff0000)>>16);
    spi_tx_buf[2] = (uint8_t)((SecAddr&0x0000ff00)>>8);
    spi_tx_buf[3] = (uint8_t)SecAddr;
    //������ɱ�־����Ϊfalse
    spi_xfer_done = false;
	  //����CS��ʹ��W25Q128FV
	  SPIFlash_CS_LOW;
	  //�������ݴ��䣺���ͳ���4���ֽڣ���ȡ����0�ֽ�
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, SPIFLASH_CMD_LENGTH, spi_rx_buf, 0));
	  //�ȴ�SPI�������
    while(!spi_xfer_done);
	  //����CS���ͷ�W25Q128FV
    SPIFlash_CS_HIGH;
	  //�ȴ�W25Q128FV��ɲ���
    SpiFlash_Wait_Busy();
}
/*****************************************************************************
** ��  ����ȫƬ����W25Q128FV��ȫƬ���������ʱ�����ֵΪ��40��
** ��  ������
** ����ֵ����
******************************************************************************/
void SPIFlash_Erase_Chip(void)
{
		//����дʹ������
    SpiFlash_Write_Enable();
		//ȫƬ��������
		//spi_tx_buf[0] = SPIFlash_ChipErase;
	  //������ɱ�־����Ϊfalse
    spi_xfer_done = false;
	  //����CS��ʹ��W25Q128FV
	  SPIFlash_CS_LOW;
	  //�������ݴ��䣺���ͳ���1���ֽڣ���ȡ����0�ֽ�
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 1, spi_rx_buf, 0));
    while(!spi_xfer_done);
	  //����CS���ͷ�W25Q128FV
	  SPIFlash_CS_HIGH;
    //�ȴ�W25Q128FV��ɲ���
    SpiFlash_Wait_Busy();
}
/*****************************************************************************
** ��  ������ָ���ĵ�ַд������,���д��ĳ��Ȳ��ܳ����õ�ַ����ҳ���ʣ��ռ�
**         *pBuffer:ָ���д������ݻ���
**         WriteAddr:д�����ʼ��ַ
**         WriteBytesNum:д����ֽ�����һ�����256���ֽ�
** ����ֵ��RET_SUCCESS
******************************************************************************/
uint8_t SpiFlash_Write_Page(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t size)
{
	  //���д������ݳ����Ƿ�Ϸ���д�볤�Ȳ��ܳ���ҳ��Ĵ�С
	  if (size > (SPIFlash_PAGE_SIZE - (WriteAddr%SPIFlash_PAGE_SIZE)))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
	  if (size == 0) return NRF_ERROR_INVALID_LENGTH;
	
    //����дʹ������
    SpiFlash_Write_Enable();
    //ҳ�������
		//spi_tx_buf[0] = SPIFlash_PageProgram;
		//24λ��ַ���ߵ�ַ��ǰ
    spi_tx_buf[1] = (uint8_t)((WriteAddr&0x00ff0000)>>16);
    spi_tx_buf[2] = (uint8_t)((WriteAddr&0x0000ff00)>>8);
    spi_tx_buf[3] = (uint8_t)WriteAddr;
	  spi_tx_buf[4] = *pBuffer;
	  
	
	  //����CS��ʹ��W25Q128FV
	  SPIFlash_CS_LOW;
	  //������ɱ�־����Ϊfalse
		spi_xfer_done = false;
	  //�������ݴ���
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, SPIFLASH_CMD_LENGTH+1, spi_rx_buf, 0));
	  //�ȴ�SPI�������
    while(!spi_xfer_done);

	
	  //������ɱ�־����Ϊfalse
		spi_xfer_done = false;
	  //�������ݴ���
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, pBuffer+1, size-1, spi_rx_buf, 0));
	  //�ȴ�SPI�������
    while(!spi_xfer_done);
	  //����CS���ͷ�W25Q128FV
		SPIFlash_CS_HIGH;
	  //�ȴ�W25Q128FV��ɲ���
		SpiFlash_Wait_Busy();

    return NRF_SUCCESS;
}
/*****************************************************************************
** ��  ������ָ���ĵ�ַд�����ݣ���д����ҳ
**         *pBuffer:ָ���д�������
**         WriteAddr:д�����ʼ��ַ
**         size:д����ֽ���
** ����ֵ��RET_SUCCESS
******************************************************************************/
uint8_t SpiFlash_Write_Buf(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t size)
{
    uint32_t PageByteRemain = 0;
	//������ʼ��ַ����ҳ���ʣ��ռ�
    PageByteRemain = SPIFlash_PAGE_SIZE - WriteAddr%SPIFlash_PAGE_SIZE;
	//�����̵����ݳ��Ȳ�����ҳ���ʣ��ռ䣬������ݳ��ȵ���size
    if(size <= PageByteRemain)
    {
        PageByteRemain = size;
    }
	//�ִα�̣�ֱ�����е����ݱ�����
    while(true)
    {
        //���PageByteRemain���ֽ�
		SpiFlash_Write_Page(pBuffer,WriteAddr,PageByteRemain);
		//��������ɣ��˳�ѭ��
        if(size == PageByteRemain)
        {
            break;
        }
        else
        {
            //������ȡ���ݵĻ����ַ
			pBuffer += PageByteRemain;
			//�����̵�ַ
            WriteAddr += PageByteRemain;
			//���ݳ��ȼ�ȥPageByteRemain
            size -= PageByteRemain;
			//�����´α�̵����ݳ���
            if(size > SPIFlash_PAGE_SIZE)
            {
                PageByteRemain = SPIFlash_PAGE_SIZE;
            }
            else
            {
                PageByteRemain = size;
            }
        }
    }
    return NRF_SUCCESS;
}
/*****************************************************************************
** ��  ������ָ���ĵ�ַ����ָ�����ȵ�����
** ��  ����pBuffer��ָ���Ŷ������ݵ��׵�ַ       
**         ReadAddr�����������ݵ���ʼ��ַ
**         size���������ֽ�����ע��size���ܳ���pBuffer�Ĵ�С��������������
** ����ֵ��
******************************************************************************/
uint8_t SpiFlash_Read(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t size)
{
		uint8_t read_size;
	  //spi_tx_buf[0] = SPIFlash_ReadData;
		
		//24λ��ַ���ߵ�ַ��ǰ
    spi_tx_buf[1] = (uint8_t)((ReadAddr&0x00ff0000)>>16);
    spi_tx_buf[2] = (uint8_t)((ReadAddr&0x0000ff00)>>8);
    spi_tx_buf[3] = (uint8_t)ReadAddr;
	
	  //����CS��ʹ��W25Q128FV
	  SPIFlash_CS_LOW;
	  //������ɱ�־����Ϊfalse
		spi_xfer_done = false;
	  //�������ݴ���
	  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, SPIFLASH_CMD_LENGTH, spi_rx_buf, 0));
	  //�ȴ�SPI�������
    while(!spi_xfer_done);
	  //��ʼ��ȡ����
	  while(size!=0)
		{
			if(size<=SPI_TXRX_MAX_LEN)
			{
				read_size = size;
				size = 0;
			}
			else
			{
				read_size = SPI_TXRX_MAX_LEN;
				size -= SPI_TXRX_MAX_LEN;
			}
			//������ɱ�־����Ϊfalse
		  spi_xfer_done = false;
	    //�������ݴ���
	    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, spi_tx_buf, 0, pBuffer, read_size));
	    //�ȴ�SPI�������
      while(!spi_xfer_done);
			pBuffer += read_size;
		}
	  //����CS���ͷ�W25Q128FV
    SPIFlash_CS_HIGH;

    return NRF_SUCCESS;
}
//SPI�¼�������
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
  //����SPI�������  
	spi_xfer_done = true;
}

/*****************************************************************************
** ��  ����������������W25Q128�Ĺܽ�
** ��  �Σ���
** ����ֵ����
******************************************************************************/
void SPI_Flash_Init(void)
{
    //��������SPIƬѡ������Ϊ���
	  nrf_gpio_cfg_output(SPI_SS_PIN);
	  //����CS
	  SPIFlash_CS_HIGH;
	  //ʹ��Ĭ�����ò�����ʼ��SPI���ýṹ��
	  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	  //��дSPI�ź����ӵ���������
    spi_config.ss_pin   = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
	  //��ʼ��SPI
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}
/********************************************END FILE*******************************************/
