#include "imu605_485.h"
#include "usart.h"


uint8_t rx_buffer[11];

IMU_t imu;

extern osSemaphoreId IMU_Chassis_BinarySemHandle;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		IMU_Read_Euler(rx_buffer);
	}
}

void imu_task(void const * argument)
{
	
	while(1)
	{
		IMU_Request_Data(0x003D,0x0003);
		osDelay(3);
	}
}



void IMU_Unlock_Instruct(void)
{
	uint8_t unlock_buffer[8] = {0x50, 0x06, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1};
	for (int i = 0; i < 8; i++)
	{
		HAL_UART_Transmit(&huart1, (unlock_buffer + i), 1, HAL_MAX_DELAY);
	}
}

void IMU_Save_Instruct(void)
{
	uint8_t save_buffer[8] = {0x50, 0x06, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B};
	for (int i = 0; i < 8; i++)
	{
		HAL_UART_Transmit(&huart1, (save_buffer + i), 1, HAL_MAX_DELAY);
	}
}

uint16_t IMU_Request_Data(uint16_t ADDR, uint16_t LEN)
{
	//	__HAL_UART_DISABLE_IT(&huart1,UART_IT_RXNE);
	uint8_t request_buffer[8];

	uint8_t Modbus_Addr = 0x50;
	uint8_t Modbus_Read = 0x03;

	uint8_t ADDRH, ADDRL;
	uint8_t LENH, LENL;
	uint8_t CRCH, CRCL;
	
	uint16_t CRC16;

	ADDRH = ADDR >> 8;
	ADDRL = ADDR;

	LENH = LEN >> 8;
	LENL = LEN;

	request_buffer[0] = Modbus_Addr;
	request_buffer[1] = Modbus_Read;
	request_buffer[2] = ADDRH;
	request_buffer[3] = ADDRL;
	request_buffer[4] = LENH;
	request_buffer[5] = LENL;
	
	CRC16 = __CRC16(request_buffer,6);
	
	CRCH = CRC16 >> 8;
  CRCL = CRC16;
	
	request_buffer[6] = CRCH;
	request_buffer[7] = CRCL;

//		while ((USART1->SR & 0X40) == 0);	 /* 等待上一个字符发送完成 */
//		USART1->DR = (uint8_t)(*(request_buffer + i)); /* 将要发送的字符写入到DR寄存器 */
		HAL_UART_Transmit(&huart1,request_buffer,8,HAL_MAX_DELAY);
	//	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
	
	return CRC16;
}


uint16_t __CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint8_t uIndex;
    int i = 0;
    uchCRCHi = 0xFF;
    uchCRCLo = 0xFF;
    for (; i<usDataLen; i++)
    {
        uIndex = uchCRCHi ^ puchMsg[i];
        uchCRCHi = uchCRCLo ^ __auchCRCHi[uIndex];
        uchCRCLo = __auchCRCLo[uIndex] ;
    }
    return (uint16_t)(((uint16_t)uchCRCHi << 8) | (uint16_t)uchCRCLo) ;
}





bool_t IMU_Read_Euler(uint8_t *rx_buffer)
{
	static uint16_t CRC16 = 0x0000;
	
	if(rx_buffer[0] != 0x50)
	{
		return IMU_Error;
	}
	if(rx_buffer[1] != 0x03)
	{
		return IMU_Error;
	}
	if(rx_buffer[2] != 0x06)
	{
		return IMU_Error;
	}
//	if (rx_cnt < 11)
//	{
//		return IMU_Error;
//	}
	else
	{
		CRC16 = __CRC16(rx_buffer,9);
		if(CRC16 != ((rx_buffer[9] << 8) | (rx_buffer[10])))
		{
//			rx_cnt=0;
			return IMU_Error;
		}
		imu.Euler.roll = (int16_t)((int16_t)rx_buffer[3] << 8 | rx_buffer[4]);
		imu.Euler.pitch = (int16_t)((int16_t)rx_buffer[5] << 8 | rx_buffer[6]);
		imu.Euler.yaw = (int16_t)((int16_t)rx_buffer[7] << 8 | rx_buffer[8]);

		imu.Euler.roll = imu.Euler.roll / 32768.0f * 180.0f;
		imu.Euler.pitch = imu.Euler.pitch / 32768.0f * 180.0f;
		imu.Euler.yaw = imu.Euler.yaw / 32768.0f * 180.0f;
		
//		rx_cnt=0;
		return IMU_OK;
	}
}



