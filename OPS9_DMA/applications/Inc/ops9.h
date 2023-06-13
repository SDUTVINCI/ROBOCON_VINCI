#ifndef __OPS_9_H_
#define __OPS_9_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "include.h"

#define OPS_READY_TIME 16000
	
typedef struct
{
	__packed struct
	{
		fp32 yaw;			//Æ«º½½Ç
		fp32 pitch;   //¸©Ñö½Ç
		fp32 roll;   //·­¹ö½Ç
	}Euler;
	
	__packed struct
	{
		fp32 pos_x;
		fp32 pos_y;
	}Position;
	
	__packed struct
	{
		fp32 yaw_velocity;
	}Euler_velocity;
	
}IMU_t;

void Data_Analyse(uint8_t rec);

void stract(char str1[],char str2[],int num);
void Update_x(float new_x);
void Update_y(float new_y);
void Update_yaw(float new_yaw);

#ifdef __cpluscplus
}
#endif

#endif
