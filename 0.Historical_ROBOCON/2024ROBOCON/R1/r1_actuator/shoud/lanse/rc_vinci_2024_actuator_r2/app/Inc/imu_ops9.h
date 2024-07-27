#ifndef __IMU_OPS_9_H_
#define __IMU_OPS_9_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "include.h"

#define OPS_READY_TIME 16000
	
#define huart_ops9 huart2
	
typedef struct
{
	__packed struct
	{
		fp32 yaw;			//??o???
		fp32 pitch;   //??????
		fp32 roll;   //¡¤-1???
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
	
}ops9_t;

void Data_Analyse(uint8_t rec);

void stract(char str1[],char str2[],int num);
void Update_x(UART_HandleTypeDef *huart,float new_x);
void Update_y(UART_HandleTypeDef *huart,float new_y);
void Update_yaw(UART_HandleTypeDef *huart,float new_yaw);

#ifdef __cpluscplus
}
#endif

#endif
