

#ifndef __IMU605_H_
#define __IMU605_H_

#ifdef __cpluscplus
extern "C"
{
#endif

#include "include.h"
#include <stdio.h>
	
	
#define huart_101ct huart5
	
typedef struct
{
	__packed struct
	{
		fp32 X;
		fp32 Y;
		fp32 Z;
	}Accel;

	__packed struct
	{
		fp32 X;
		fp32 Y;
		fp32 Z;
	}Gyro;
	
	__packed struct
	{
		fp32 X;
		fp32 Y;
		fp32 Z;
	}Magnet;
	
	__packed struct
	{
		//fp32 yaw;    //偏航角  前进的偏移
		fp32 pitch;  //俯仰角	 前后的上下摆动
		fp32 roll;   //翻滚角  左右的上下摆动
	}Euler;//欧拉角
	
	__packed struct
	{
		fp32 quat[4];
	}Quaternion;
	
	__packed struct
	{
		fp32 temp;
	}Other_Data;
	
}HWT101_t;

extern HWT101_t hwt101ct;

#define	Accel_Type  		  0x51
#define	Gyro_Type  			0x52
#define	Magnet_Type  		0x54
#define	Euler_Type  		 	0x53
#define	Quaternion_Type 0x59

#define IMU_OK 1
#define IMU_Error 0

void CopeSerial2Data(unsigned char ucData);


bool_t IMU_Read_Euler(uint8_t *rx_buffer);
uint8_t __SUMCRC(uint8_t *puchMsg, uint16_t usDataLen);

void IMU_Save(UART_HandleTypeDef *huart);
void IMU_Unlock(UART_HandleTypeDef *huart);
void IMU_Reboot(UART_HandleTypeDef *huart);
void IMU_Zero(UART_HandleTypeDef *huart);
void IMU_Write_Register(UART_HandleTypeDef *huart,uint8_t reg_addr,uint16_t tx_data);

#ifdef __cpluscplus
}
#endif

#endif
