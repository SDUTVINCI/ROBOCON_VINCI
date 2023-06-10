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



#include "imu605.h"

IMU_t imu;

uint8_t rx_buffer[1];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		IMU_Read_Euler(rx_buffer);
	}
}

uint8_t __SUMCRC(uint8_t *puchMsg, uint16_t usDataLen)
{
    int16_t i = 0;
		uint8_t uchSUMCRC = 0x00;
    for (; i < usDataLen; i++)
    {
			uchSUMCRC += puchMsg[i];
    }
    return uchSUMCRC;
}


bool_t IMU_Read_Euler(uint8_t *rx_buffer)
{
	static uint8_t imu_buffer[50];
	static int16_t rx_cnt = 0;
	static uint16_t SUMCRC;
	
	imu_buffer[rx_cnt++] = *rx_buffer;
	if(imu_buffer[0] != 0x55)
	{
		rx_cnt=0;
		return IMU_Error;
	}
	if(imu_buffer[1] != 0x53)
	{
		rx_cnt=0;
		return IMU_Error;
	}
	if (rx_cnt < 11)
	{
		return IMU_Error;
	}
	else
	{
		SUMCRC = __SUMCRC(imu_buffer,10);
		if(SUMCRC != imu_buffer[10])
		{
			rx_cnt=0;
			return IMU_Error;
		}
		imu.Euler.roll = (int16_t)((int16_t)imu_buffer[3] << 8 | imu_buffer[2]);
		imu.Euler.pitch = (int16_t)((int16_t)imu_buffer[5] << 8 | imu_buffer[4]);
		imu.Euler.yaw = (int16_t)((int16_t)imu_buffer[7] << 8 | imu_buffer[6]);

		imu.Euler.roll = imu.Euler.roll / 32768.0f * 180.0f;
		imu.Euler.pitch = imu.Euler.pitch / 32768.0f * 180.0f;
		imu.Euler.yaw = imu.Euler.yaw / 32768.0f * 180.0f;
		
		rx_cnt=0;
		return IMU_OK;
	}
}


