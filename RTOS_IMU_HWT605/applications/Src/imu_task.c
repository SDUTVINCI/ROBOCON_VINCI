//=====================================================================================================
// imu605.c
//=====================================================================================================
// See: https://github.com/tungchiahui
//
// Date : 28/05/2023		             
//
// Organization : 机电创新学会 Vinci机器人队
//
// Author : Tung Chia-hui
//
// Notes : Initial release
//
//=====================================================================================================


#include "imu_task.h"

IMU_t imu;

bool_t imu_flag;

uint8_t rx_buffer[1];

uint8_t imu_buffer[11];


void imu_task(void const * argument)
{
	while(1)
	{
		HAL_UART_Receive(&huart1,rx_buffer,1,portMAX_DELAY);
		IMU_Read_Data(rx_buffer);
	}
}


void IMU_Read_Data(uint8_t *rx_buffer)
{
	static int16_t imu_cnt = 0;
	imu_buffer[imu_cnt] = *rx_buffer;
	if(imu_buffer[0] == 0x55)
	{
		imu_cnt++;
		if(imu_flag == 1)
		{
		if(imu_buffer[1] == Accel_Type)
		{
			imu.Accel.X = (int16_t)((int16_t)imu_buffer[3]<<8|imu_buffer[2]);
			imu.Accel.Y = (int16_t)((int16_t)imu_buffer[5]<<8|imu_buffer[4]);
			imu.Accel.Z = (int16_t)((int16_t)imu_buffer[7]<<8|imu_buffer[6]);
			imu.Other_Data.temp = (int16_t)((int16_t)imu_buffer[9]<<8|imu_buffer[8]);
			
			imu.Accel.X = imu.Accel.X / 32768.0f * 16.0f * 9.8f;
			imu.Accel.Y = imu.Accel.Y / 32768.0f * 16.0f * 9.8f;
			imu.Accel.Z = imu.Accel.Z / 32768.0f * 16.0f * 9.8f;
			imu.Other_Data.temp = imu.Other_Data.temp / 100.0f;
			
			imu.Accel.Z -= 9.8f;
		}
		else if(imu_buffer[1] == Gyro_Type)
		{
			imu.Gyro.X = (int16_t)((int16_t)imu_buffer[3]<<8|imu_buffer[2]);
			imu.Gyro.Y = (int16_t)((int16_t)imu_buffer[5]<<8|imu_buffer[4]);
			imu.Gyro.Z = (int16_t)((int16_t)imu_buffer[7]<<8|imu_buffer[6]);
			
			imu.Gyro.X = imu.Gyro.X / 32768.0f * 2000.0f;
			imu.Gyro.Y = imu.Gyro.Y / 32768.0f * 2000.0f;
			imu.Gyro.Z = imu.Gyro.Z / 32768.0f * 2000.0f;
		}
		else if(imu_buffer[1] == Magnet_Type)
		{
			imu.Magnet.X = (int16_t)((int16_t)imu_buffer[3]<<8|imu_buffer[2]);
			imu.Magnet.Y = (int16_t)((int16_t)imu_buffer[5]<<8|imu_buffer[4]);
			imu.Magnet.Z = (int16_t)((int16_t)imu_buffer[7]<<8|imu_buffer[6]);
		}
		else if(imu_buffer[1] == Euler_Type)
		{
			imu.Euler.roll = (int16_t)((int16_t)imu_buffer[3]<<8|imu_buffer[2]);
			imu.Euler.pitch = (int16_t)((int16_t)imu_buffer[5]<<8|imu_buffer[4]);
			imu.Euler.yaw = (int16_t)((int16_t)imu_buffer[7]<<8|imu_buffer[6]);
			
			imu.Euler.roll = imu.Euler.roll / 32768.0f * 180.0f;
			imu.Euler.pitch = imu.Euler.pitch / 32768.0f * 180.0f;
			imu.Euler.yaw = imu.Euler.yaw / 32768.0f * 180.0f;
		}
		else if(imu_buffer[1] == Quaternion_Type)
		{
			imu.Quaternion.quat[0]=(int16_t)((int16_t)imu_buffer[3]<<8|imu_buffer[2]);
			imu.Quaternion.quat[1]=(int16_t)((int16_t)imu_buffer[5]<<8|imu_buffer[4]);
			imu.Quaternion.quat[2]=(int16_t)((int16_t)imu_buffer[7]<<8|imu_buffer[6]);
			imu.Quaternion.quat[3]=(int16_t)((int16_t)imu_buffer[9]<<8|imu_buffer[8]);
			
			for(int i = 0;i < 4;i++)
			{
				imu.Quaternion.quat[i] /= 32768.0f;
			}
		}
		imu_flag = 0;
		for(int i = 0;i < 11;i++)
		{
			imu_buffer[i] = 0x00;
		}
	}
	}
	else
	{
		IMU_ZERO:
		imu_cnt = 0;
		return;
	}
	if(imu_cnt >= 11)
	{
		imu_flag = 1;
		goto IMU_ZERO;
	}
	
}

