#ifndef _IMU_H
#define _IMU_H

#include "Mahony.h"
#include "w25q128.h"
#include <stdio.h>
#include <math.h>
#include "nrf_delay.h"

extern short aacx,aacy,aacz;		
extern short gyrox,gyroy,gyroz;	
extern float Roll,Pitch,Yaw;

typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;

	float Pitch_v;
	float Roll_v;
	float Yaw_v;

	float ax;
	float ay;
	float az;

} IMU_Info;

extern IMU_Info imu;
extern float IMU_Temperature;

void IMU_Init(void);
void IMU_Update(void);
void IMU_readGyro_Acc(int16_t *gyro, int16_t *acce, int16_t *mag);
void IMU_Init_Offset(void);
float safe_asin(float v);

#endif
