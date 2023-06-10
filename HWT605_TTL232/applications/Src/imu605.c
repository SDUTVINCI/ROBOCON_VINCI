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
#include <stdio.h>


IMU_t imu;

int16_t rx_cnt = 0;

uint8_t rx_buffer[121];

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
	static uint16_t SUMCRC;
	uint8_t rx_buffer_pre;
	for(int i = 0;i < 121;i++)
	{
		if(rx_buffer_pre == 0x55 && rx_buffer[i] == 0x53)
		{
			rx_buffer_pre = rx_buffer[i];
			rx_cnt = i - 1;
		}
	}
	
	if(rx_buffer[rx_cnt] != 0x55)
	{
		return IMU_Error;
	}
	if(rx_buffer[rx_cnt + 1] != 0x53)
	{
		return IMU_Error;
	}
	else
	{
		SUMCRC = __SUMCRC(&(rx_buffer[rx_cnt]),10);
		if(SUMCRC != rx_buffer[rx_cnt + 10])
		{
			return IMU_Error;
		}
		imu.Euler.roll = (int16_t)((int16_t)rx_buffer[rx_cnt + 3] << 8 | rx_buffer[rx_cnt + 2]);
		imu.Euler.pitch = (int16_t)((int16_t)rx_buffer[rx_cnt + 5] << 8 | rx_buffer[rx_cnt + 4]);
		imu.Euler.yaw = (int16_t)((int16_t)rx_buffer[rx_cnt + 7] << 8 | rx_buffer[rx_cnt + 6]);

		imu.Euler.roll = imu.Euler.roll / 32768.0f * 180.0f;
		imu.Euler.pitch = imu.Euler.pitch / 32768.0f * 180.0f;
		imu.Euler.yaw = imu.Euler.yaw / 32768.0f * 180.0f;
		
		return IMU_OK;
	}
}


