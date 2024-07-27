//=====================================================================================================
// hwt101ct_rs232.c
//=====================================================================================================
// See: https://github.com/tungchiahui
//
// Date : 28/05/2023		             
//
// Organization : 机电创新学会EMIS Vinci机器人队
//
// Author : Tung Chia-hui
//
// Notes : Initial release
//
//=====================================================================================================

#include "hwt101ct_rs232.h"

HWT101CT_t hwt101ct;
uint8_t rx_buffer[1];


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		IMU_Read_Euler(rx_buffer);
	}
}


bool IMU_Read_Euler(uint8_t *msg_data)
{
	static uint8_t CheckSum;
    static uint8_t msg_data_pre;
	static uint8_t buffer[11];
	static int16_t rx_cnt = 1;
	static uint8_t finded_flag;

    if(msg_data_pre == 0x55 && *msg_data == 0x53)
    {
        finded_flag = 1;
    }
    if(finded_flag == 1)
    {
        buffer[0] = msg_data_pre;
        buffer[rx_cnt++] = *msg_data;
        if(rx_cnt == 11)
        {
            finded_flag = 0;
			rx_cnt = 1;
            msg_data_pre = 0;

            CheckSum = __SUMCRC(buffer,10);
            if(CheckSum != buffer[10])
		    {
		    	return false;
		    }
//		    hwt101ct.Euler.roll = (int16_t)((int16_t)buffer[3] << 8 | buffer[2]);
//		    hwt101ct.Euler.pitch = (int16_t)((int16_t)buffer[5] << 8 | buffer[4]);
		    hwt101ct.Euler.yaw = (int16_t)((int16_t)buffer[7] << 8 | buffer[6]);

//		    hwt101ct.Euler.roll = hwt101ct.Euler.roll / 32768.0f * 180.0f;
//		    hwt101ct.Euler.pitch = hwt101ct.Euler.pitch / 32768.0f * 180.0f;
		    hwt101ct.Euler.yaw = hwt101ct.Euler.yaw / 32768.0f * 180.0f;
		
		    return true;
        }
    }
    else
    {
        msg_data_pre = *msg_data;
    }
    return false;
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




