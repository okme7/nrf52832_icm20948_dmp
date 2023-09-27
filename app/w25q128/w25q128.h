/*----------------------------------------------------------------------------------------------------
** Created by        : [艾克姆]
** Created date      : 2018-12-24
** Version           : 1.0
** Descriptions      : W25Q128 SPI Flash驱动程序头文件
**---------------------------------------------------------------------------------------------------*/
#ifndef __W25Q128_H
#define __W25Q128_H
#include <stdint.h>


typedef struct {
	int16_t Gyro_X;
	int16_t Accel_X;
	int16_t Mag_X;
	int16_t Gyro_Y;
	int16_t Accel_Y;
	int16_t Mag_Y;
	int16_t Gyro_Z;
	int16_t Accel_Z;
	int16_t Mag_Z;
}IMU_RAW_DATA;

typedef enum
{
	ub_0 = 0 << 4,
	ub_1 = 1 << 4,
	ub_2 = 2 << 4,
	ub_3 = 3 << 4
} userbank;

typedef enum
{
	_250dps,
	_500dps,
	_1000dps,
	_2000dps
} gyro_full_scale;

typedef enum
{
	_2g,
	_4g,
	_8g,
	_16g
} accel_full_scale;

typedef struct
{
	float x;
	float y;
	float z;
} axises;

typedef enum
{
	power_down_mode = 0,
	single_measurement_mode = 1,
	continuous_measurement_10hz = 2,
	continuous_measurement_20hz = 4,
	continuous_measurement_50hz = 6,
	continuous_measurement_100hz = 8
} operation_mode;
/*****************************************************************************
**
*****************************************************************************/
//SPI引脚定义
#define  SPI_SS_PIN     13
#define  SPI_SCK_PIN    12
#define  SPI_MISO_PIN   14
#define  SPI_MOSI_PIN   11


#define    SPIFlash_CS_LOW    nrf_gpio_pin_clear(SPI_SS_PIN)   //片选输出低电平：使能芯片
#define    SPIFlash_CS_HIGH   nrf_gpio_pin_set(SPI_SS_PIN)     //片选输出高电平：取消片选


//ICM20948 ID
#define ICM20948				0xea

//SPI ICM20948命令定义
//#define REG_BANK_SEL 		0X7F

//#define ODR_ALIGN_EN 		0X09
#define WHO_AM_I_s				0
#define PWR_MGMT_1			0X06
#define USER_CTRL				0X03
//#define GYRO_SMPLRT_DIV	0X00
#define GYRO_CONFIG_1		0X01
#define GYRO_CONFIG_2		0X02

#define ACCEL_SMPLRT_DIV_1	0X10
#define ACCEL_SMPLRT_DIV_2	0X10
#define ACCEL_CONFIG				0X14

//#define ACCEL_XOUT_H 		0X2D
//#define ACCEL_XOUT_L 		0X2E
//#define ACCEL_YOUT_H 		0X2F
//#define ACCEL_YOUT_L 		0X30
//#define ACCEL_ZOUT_H 		0X31
//#define ACCEL_ZOUT_L 		0X32

//#define GYRO_XOUT_H 		0X33
//#define GYRO_XOUT_L 		0X34
//#define GYRO_YOUT_H 		0X35
//#define GYRO_YOUT_L 		0X36
//#define GYRO_ZOUT_H 		0X37
//#define GYRO_ZOUT_L 		0X38

/* Defines */
#define READ							0x80
#define WRITE							0x00

/* ICM-20948 Registers */
#define ICM20948_ID						0xEA
//#define REG_BANK_SEL					0x7F

// USER BANK 0
#define B0_WHO_AM_I						0x00		
#define B0_USER_CTRL					0x03
#define B0_LP_CONFIG					0x05
#define B0_PWR_MGMT_1					0x06
#define B0_PWR_MGMT_2					0x07
#define B0_INT_PIN_CFG					0x0F		
#define B0_INT_ENABLE					0x10
#define B0_INT_ENABLE_1					0x11
#define B0_INT_ENABLE_2					0x12
#define B0_INT_ENABLE_3					0x13
#define B0_I2C_MST_STATUS				0x17		
#define B0_INT_STATUS					0x19		
#define B0_INT_STATUS_1					0x1A
#define B0_INT_STATUS_2					0x1B
#define B0_INT_STATUS_3					0x1C
#define B0_DELAY_TIMEH					0x28
#define B0_DELAY_TIMEL					0x29
#define B0_ACCEL_XOUT_H					0x2D		
#define B0_ACCEL_XOUT_L					0x2E		
#define B0_ACCEL_YOUT_H					0x2F		
#define B0_ACCEL_YOUT_L					0x30		
#define B0_ACCEL_ZOUT_H					0x31		
#define B0_ACCEL_ZOUT_L					0x32	
#define B0_GYRO_XOUT_H					0x33	
#define B0_GYRO_XOUT_L					0x34
#define B0_GYRO_YOUT_H					0x35
#define B0_GYRO_YOUT_L					0x36
#define B0_GYRO_ZOUT_H					0x37
#define B0_GYRO_ZOUT_L					0x38
#define B0_TEMP_OUT_H					0x39		
#define B0_TEMP_OUT_L					0x3A
#define B0_EXT_SLV_SENS_DATA_00			0x3B
#define B0_EXT_SLV_SENS_DATA_01			0x3C
#define B0_EXT_SLV_SENS_DATA_02			0x3D
#define B0_EXT_SLV_SENS_DATA_03			0x3E
#define B0_EXT_SLV_SENS_DATA_04			0x3F
#define B0_EXT_SLV_SENS_DATA_05			0x40
#define B0_EXT_SLV_SENS_DATA_06			0x41
#define B0_EXT_SLV_SENS_DATA_07			0x42
#define B0_EXT_SLV_SENS_DATA_08			0x43
#define B0_EXT_SLV_SENS_DATA_09			0x44
#define B0_EXT_SLV_SENS_DATA_10			0x45
#define B0_EXT_SLV_SENS_DATA_11			0x46
#define B0_EXT_SLV_SENS_DATA_12			0x47
#define B0_EXT_SLV_SENS_DATA_13			0x48
#define B0_EXT_SLV_SENS_DATA_14			0x49
#define B0_EXT_SLV_SENS_DATA_15			0x4A
#define B0_EXT_SLV_SENS_DATA_16			0x4B
#define B0_EXT_SLV_SENS_DATA_17			0x4C
#define B0_EXT_SLV_SENS_DATA_18			0x4D
#define B0_EXT_SLV_SENS_DATA_19			0x4E
#define B0_EXT_SLV_SENS_DATA_20			0x4F
#define B0_EXT_SLV_SENS_DATA_21			0x50
#define B0_EXT_SLV_SENS_DATA_22			0x51
#define B0_EXT_SLV_SENS_DATA_23			0x52
#define B0_FIFO_EN_1					0x66	
#define B0_FIFO_EN_2					0x67
#define B0_FIFO_RST						0x68
#define B0_FIFO_MODE					0x69
#define B0_FIFO_COUNTH					0X70
#define B0_FIFO_COUNTL					0X71
#define B0_FIFO_R_W						0x72
#define B0_DATA_RDY_STATUS				0x74
#define B0_FIFO_CFG						0x76	

// USER BANK 1
#define B1_SELF_TEST_X_GYRO				0x02	
#define B1_SELF_TEST_Y_GYRO				0x03
#define B1_SELF_TEST_Z_GYRO				0x04
#define B1_SELF_TEST_X_ACCEL			0x0E	
#define B1_SELF_TEST_Y_ACCEL			0x0F
#define B1_SELF_TEST_Z_ACCEL			0x10
#define B1_XA_OFFS_H					0x14	
#define B1_XA_OFFS_L					0x15
#define B1_YA_OFFS_H					0x17
#define B1_YA_OFFS_L					0x18
#define B1_ZA_OFFS_H					0x1A
#define B1_ZA_OFFS_L					0x1B
#define B1_TIMEBASE_CORRECTION_PLL		0x28	

// USER BANK 2
#define B2_GYRO_SMPLRT_DIV				0x00	
#define B2_GYRO_CONFIG_1				0x01	
#define B2_GYRO_CONFIG_2				0x02
#define B2_XG_OFFS_USRH					0x03	
#define B2_XG_OFFS_USRL 				0x04
#define B2_YG_OFFS_USRH					0x05
#define B2_YG_OFFS_USRL					0x06
#define B2_ZG_OFFS_USRH					0x07
#define B2_ZG_OFFS_USRL					0x08
#define B2_ODR_ALIGN_EN					0x09	
#define B2_ACCEL_SMPLRT_DIV_1			0x10	
#define B2_ACCEL_SMPLRT_DIV_2			0x11		
#define B2_ACCEL_INTEL_CTRL				0x12		
#define B2_ACCEL_WOM_THR				0x13
#define B2_ACCEL_CONFIG					0x14
#define B2_ACCEL_CONFIG_2				0x15
#define B2_FSYNC_CONFIG					0x52
#define B2_TEMP_CONFIG					0x53
#define B2_MOD_CTRL_USR					0X54

// USER BANK 3
#define B3_I2C_MST_ODR_CONFIG			0x00
#define B3_I2C_MST_CTRL					0x01
#define B3_I2C_MST_DELAY_CTRL			0x02	
#define B3_I2C_SLV0_ADDR				0x03
#define B3_I2C_SLV0_REG					0x04		
#define B3_I2C_SLV0_CTRL				0x05
#define B3_I2C_SLV0_DO					0x06
#define B3_I2C_SLV1_ADDR				0x07		
#define B3_I2C_SLV1_REG					0x08		
#define B3_I2C_SLV1_CTRL				0x09
#define B3_I2C_SLV1_DO					0x0A
#define B3_I2C_SLV2_ADDR				0x0B		
#define B3_I2C_SLV2_REG					0x0C		
#define B3_I2C_SLV2_CTRL				0x0D
#define B3_I2C_SLV2_DO					0x0E
#define B3_I2C_SLV3_ADDR				0x0F		
#define B3_I2C_SLV3_REG					0x10		
#define B3_I2C_SLV3_CTRL				0x11
#define B3_I2C_SLV3_DO					0x12
#define B3_I2C_SLV4_ADDR				0x13	
#define B3_I2C_SLV4_REG					0x14		
#define B3_I2C_SLV4_CTRL				0x15
#define B3_I2C_SLV4_DO					0x16
#define B3_I2C_SLV4_DI					0x17
	
//AK9916
#define AK9916_Addr 0x0c

#define AK9916_WIA2 0x01
#define AK9916_ST1  0x10
#define AK9916_HXL  0x11
#define AK9916_HXH  0x12
#define AK9916_HYL  0x13
#define AK9916_HYH  0x14
#define AK9916_HZL  0x15
#define AK9916_HZH  0x16
#define AK9916_ST2  0x18
#define AK9916_CNTL2  0x31
#define AK9916_CNTL3  0x32
#define AK9916_TS1  0x33
#define AK9916_TS2  0x34


#define    SPIFLASH_CMD_LENGTH        0x04
#define    SPIFLASH_WRITE_BUSYBIT     0x01



#define    SPIFlash_PAGE_SIZE        256
#define    SPIFlash_SECTOR_SIZE      (1024*4)
#define    SPI_TXRX_MAX_LEN          255

#define    FLASH_BLOCK_NUMBLE         7
#define    FLASH_PAGE_NUMBLE          8

void SPI_Flash_Init(void);
uint16_t SpiFlash_ReadID(void);


uint8_t SpiFlash_Write_Page(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t size);
uint8_t SpiFlash_Read(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t size);
void SPIFlash_Erase_Sector(uint32_t SecAddr);
void SPIFlash_Erase_Chip(void);
uint8_t SpiFlash_Write_Buf(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t WriteBytesNum);

void icm20948_init(void);
uint8_t SpiICM20948_Read(void);
void IMU_GetRawData(IMU_RAW_DATA *raw_data ); 
void ICM94_AUX_ReadRegs( uint8_t slaveAddr, uint8_t readAddr, uint8_t *readData, uint8_t lens );
void SpiICM20948_Read_x(void);
uint8_t  read_single_icm20948_reg(userbank ub, uint8_t reg);
void  write_single_icm20948_reg(userbank ub,uint8_t writeAddr, uint8_t writeData);
static void select_user_bank(userbank ub);
static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len);

// 16 bits ADC value. raw data.
void icm20948_gyro_read(axises* data);	
void icm20948_accel_read(axises* data);


// Convert 16 bits ADC value to their unit.
void icm20948_gyro_read_dps(axises* data); 
void icm20948_accel_read_g(axises* data);

void icm20948_angle_read(axises* data);

void MPU_Get_Gyroscope(short* gx, short* gy, short* gz);
void MPU_Get_Accelerometer(short* ax, short* ay, short* az);
void MPU_Get_Magnetism(short* mx, short* my, short* mz);

void write_multiple_spi(uint8_t reg, uint8_t *data, uint32_t len);
void read_multiple_spi(uint8_t reg, uint8_t *buff, uint32_t len);

#endif
