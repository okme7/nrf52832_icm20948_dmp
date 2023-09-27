#include "IMU.h"

#define OFFSET_COUNT 500
#define FIFO_SIZE 10

signed short     int IMU_FIFO[9][FIFO_SIZE];
static unsigned char Wr_Index = 0; 

static float Pitch_offset;
static float Roll_offset;
static float Yaw_offset;
short aacx,aacy,aacz;		 
short gyrox,gyroy,gyroz;
short magx,magy,magz;
float IMU_Temperature = 0 ; 

IMU_Info imu;

void IMU_NewVal(int16_t *IMU_FIFO, int16_t IMU_val)
{
	IMU_FIFO[Wr_Index] = IMU_val;
}

int16_t IMU_GetAvg(int16_t *IMU_FIFO)
{
	uint8_t i;
	int32_t sum = 0;
	for (i = 0; i < FIFO_SIZE; i++)
		sum += IMU_FIFO[i];
	sum = sum / FIFO_SIZE;
	return (int16_t)sum;
}

 void Get_IMU_Values(float *IMU_values)
{
	short i = 0;
	int16_t gyro[3], acc[3], mag[3];

	IMU_readGyro_Acc(&gyro[0], &acc[0], &mag[0]);
	
	for (; i < 3; i++)
	{
		IMU_values[i] = ((float)gyro[i]) / 16.4f;
		IMU_values[3 + i] = (float)acc[i] ;
		IMU_values[6 + i] = (float)mag[i] ;

	}
}

void IMU_readGyro_Acc(int16_t *gyro, int16_t *acce, int16_t *mag)
{
	static short buf[9];
	static int16_t gx, gy, gz;
	static int16_t ax, ay, az;
	static int16_t mx, my, mz;
	
	MPU_Get_Gyroscope(&buf[0], &buf[1], &buf[2]);
	MPU_Get_Accelerometer(&buf[3], &buf[4], &buf[5]);
	MPU_Get_Magnetism(&buf[6], &buf[7], &buf[8]);

	IMU_NewVal(&IMU_FIFO[0][0], buf[0]);
	IMU_NewVal(&IMU_FIFO[1][0], buf[1]);
	IMU_NewVal(&IMU_FIFO[2][0], buf[2]);

	IMU_NewVal(&IMU_FIFO[3][0], buf[3]);
	IMU_NewVal(&IMU_FIFO[4][0], buf[4]);
	IMU_NewVal(&IMU_FIFO[5][0], buf[5]);

	IMU_NewVal(&IMU_FIFO[6][0], buf[6]);
	IMU_NewVal(&IMU_FIFO[7][0], buf[7]);
	IMU_NewVal(&IMU_FIFO[8][0], buf[8]);

	Wr_Index = (Wr_Index + 1) % FIFO_SIZE;

	gx = IMU_GetAvg(&IMU_FIFO[0][0]);
	gy = IMU_GetAvg(&IMU_FIFO[1][0]);
	gz = IMU_GetAvg(&IMU_FIFO[2][0]);

	gyro[0] = gx - Roll_offset;
	gyro[1] = gy - Pitch_offset;
	gyro[2] = gz - Yaw_offset;

	ax = IMU_GetAvg(&IMU_FIFO[3][0]);
	ay = IMU_GetAvg(&IMU_FIFO[4][0]);
	az = IMU_GetAvg(&IMU_FIFO[5][0]);

	acce[0] = ax;
	acce[1] = ay;
	acce[2] = az;

	mx = IMU_GetAvg(&IMU_FIFO[6][0]);
	my = IMU_GetAvg(&IMU_FIFO[7][0]);
	mz = IMU_GetAvg(&IMU_FIFO[8][0]);

	mag[0] = mx + 238;
	mag[1] = (my + 426) * 0.783784;
	mag[2] = (mz + 188) * 0.701280;
}

void IMU_Update(void)
{
	static float q[4];
	float Values[9];

	Get_IMU_Values(Values);
//	MahonyAHRSupdateIMU(Values[0] * 3.14159265358979f / 180, Values[1] * 3.14159265358979f / 180, Values[2] * 3.14159265358979f / 180,Values[3], Values[4], Values[5]);

	MahonyAHRSupdate(Values[0] * 3.14159265358979f / 180, Values[1] * 3.14159265358979f / 180, Values[2] * 3.14159265358979f / 180,Values[3], Values[4], Values[5],Values[6], Values[7], Values[8]);
	
	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;

	imu.ax = Values[3];
	imu.ay = Values[4];
	imu.az = Values[5];

	imu.Pitch_v = Values[0];
	imu.Roll_v = Values[1];
	imu.Yaw_v = Values[2];

	imu.Roll = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1 - 2.0f * (q[1] * q[1] + q[2] * q[2]))) * 180 / 3.14159265358979f;
	imu.Pitch = -safe_asin(2.0f * (q[0] * q[2] - q[1] * q[3])) * 180 / 3.14159265358979f;
  imu.Yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 180/3.14159265358979f;
  if(imu.Yaw >=  360  ) imu.Yaw = 0;
	if(imu.Yaw <= -360  ) imu.Yaw = 0;
//  imu.Yaw += ((double)(-gyroz + Yaw_offset) * 3.1415926f / 360 / 32.8f * 0.1 * 1.5 );        //1.5
}


void IMU_Init_Offset(void)
{
	short i;
	int tempgx = 0, tempgy = 0, tempgz = 0;
	int tempax = 0, tempay = 0, tempaz = 0;

	Pitch_offset = 0;
	Roll_offset = 0;
	Yaw_offset = 0;
	nrf_delay_ms(10);
	
	// read the mpu data for calculate the offset
	for (i = 0; i < OFFSET_COUNT; i++)
	{
		nrf_delay_ms(5);
		
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	
		
		tempgx += gyrox;
		tempgy += gyroy;
		tempgz += gyroz;

		tempax += aacx;
		tempay += aacy;
		tempaz += aacz;
		

	}
		
	Roll_offset  = tempgx / OFFSET_COUNT;
	Pitch_offset = tempgy / OFFSET_COUNT;
	Yaw_offset   = tempgz / OFFSET_COUNT;

	printf("Pitch_offset = %f,Roll_offset = %f,Yaw_offset = %f\r\n", Pitch_offset, Roll_offset, Yaw_offset);
}

void IMU_Init(void)
{

	icm20948_init();
	
//	IMU_Init_Offset();
}


IMU_Info *IMU_GetInfo(void) { return &imu; }

//arcsin
float safe_asin(float v)
{
	if (isnan(v))  return 0.0f;
	if (v >= 1.0f) return 3.14159265358979f/2;
	if (v <= -1.0f)return -3.14159265358979f/2;
	return asin(v);
}

