#ifndef __IMU_USE__
#define __IMU_USE__

#include "main.h"

//bool_t IMU_Read_Euler(uint8_t * Data);


#include "struct_typedef.h"

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
		float yaw;    //ƫ����  ǰ����ƫ��
		float pitch;  //������	 ǰ������°ڶ�
		float roll;   //������  ���ҵ����°ڶ�
	}Euler;//ŷ����
	
	__packed struct
	{
		fp32 quat[4];
	}Quaternion;
	
	__packed struct
	{
		fp32 temp;
	}Other_Data;
	
}IMU_t;

#define	Accel_Type  		  0x51
#define	Gyro_Type  			0x52
#define	Magnet_Type  		0x54
#define	Euler_Type  		 	0x53
#define	Quaternion_Type 0x59

#define IMU_OK 1
#define IMU_Error 0

bool_t IMU_Read_Euler(uint8_t *rx_buffer);
uint8_t __SUMCRC(uint8_t *puchMsg, uint16_t usDataLen);


#endif


