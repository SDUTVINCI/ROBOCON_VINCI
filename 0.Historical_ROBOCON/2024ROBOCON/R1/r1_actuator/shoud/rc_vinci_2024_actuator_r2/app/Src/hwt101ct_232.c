


#include "hwt101ct_232.h"


#include <string.h>

HWT101_t hwt101ct;

int16_t rx_cnt = 0;

uint8_t imu_rx_buffer[44];



 

 

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART5)
	{
		IMU_Read_Euler(imu_rx_buffer);
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
	uint8_t rx_buffer_pre;
	static uint16_t SUMCRC;
	for(int i = 0;i < 44;i++)
	{
		if(rx_buffer_pre == 0x55 && rx_buffer[i] == 0x53)
		{
			rx_cnt = i - 1;
			goto DATA_HEADER_CORRECT;
		}
		rx_buffer_pre = rx_buffer[i];
	}
	DATA_HEADER_CORRECT:
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
//		imu.Euler.roll = (int16_t)((int16_t)rx_buffer[rx_cnt + 3] << 8 | rx_buffer[rx_cnt + 2]);
//		imu.Euler.pitch = (int16_t)((int16_t)rx_buffer[rx_cnt + 5] << 8 | rx_buffer[rx_cnt + 4]);
		hwt101ct.Euler.yaw = (int16_t)((int16_t)rx_buffer[rx_cnt + 7] << 8 | rx_buffer[rx_cnt + 6]);

//		imu.Euler.roll = imu.Euler.roll / 32768.0f * 180.0f;
//		imu.Euler.pitch = imu.Euler.pitch / 32768.0f * 180.0f;
		hwt101ct.Euler.yaw = hwt101ct.Euler.yaw / 32768.0f * 180.0f;
		
		return IMU_OK;
	}
}


void IMU_Write_Register(UART_HandleTypeDef *huart,uint8_t reg_addr,uint16_t tx_data)
{
		uint8_t tx_buffer[5];
    tx_buffer[0] = 0xFF;
    tx_buffer[1] = 0xAA;
    tx_buffer[2] = reg_addr;
    tx_buffer[3] = tx_data;
    tx_buffer[4] = tx_data >> 8;
    HAL_UART_Transmit_DMA(huart,tx_buffer,5);
}

void IMU_Zero(UART_HandleTypeDef *huart)
{
	uint8_t zero_buffer[5] = {0xff, 0xaa, 0x76, 0x00, 0x00};
	HAL_UART_Transmit_DMA(huart,zero_buffer,5);
}


void IMU_Unlock(UART_HandleTypeDef *huart)
{
	uint8_t unlock_buffer[5] = {0xff, 0xaa, 0x69, 0x88, 0xb5};
	HAL_UART_Transmit_DMA(huart,unlock_buffer,5);
}

void IMU_Save(UART_HandleTypeDef *huart)
{
	uint8_t save_buffer[5] = {0xff, 0xaa, 0x00, 0x00, 0x00};
	HAL_UART_Transmit_DMA(huart,save_buffer,5);
}

void IMU_Reboot(UART_HandleTypeDef *huart)
{
	uint8_t reboot_buffer[5] = {0xff, 0xaa, 0x00, 0xff, 0x00};
	HAL_UART_Transmit_DMA(huart,reboot_buffer,5);
}



