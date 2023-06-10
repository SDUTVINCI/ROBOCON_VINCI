#include "hwt605_modbus.h"
#include <stdio.h>


uint8_t rx_buffer[200];

uint8_t imu_buffer[50];

IMU_t imu;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
//		for(int i = 0;i < 11;i++)
//		{
//			printf("%x",rx_buffer[i]);
//		}
		IMU_Read_Euler(rx_buffer);
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

//		while ((USART1->SR & 0X40) == 0);	 /* �ȴ���һ���ַ�������� */
//		USART1->DR = (uint8_t)(*(request_buffer + i)); /* ��Ҫ���͵��ַ�д�뵽DR�Ĵ��� */
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

bool_t IMU_Read_Euler(uint8_t *rx_buffer)  //����ʽ
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


//bool_t IMU_Read_Euler(uint8_t *rx_buffer)  //����
//{
//	static int16_t rx_cnt = 0;
//	static uint16_t CRC16 = 0x0000;
//	
//	imu_buffer[rx_cnt++] = *rx_buffer;
//	
//	if(imu_buffer[0] != 0x50)
//	{
//		rx_cnt=0;
//		return IMU_Error;
//	}
//	if(imu_buffer[1] != 0x03)
//	{
//		rx_cnt=0;
//		return IMU_Error;
//	}
//	if(imu_buffer[2] != 0x06)
//	{
//		rx_cnt=0;
//		return IMU_Error;
//	}
//	if (rx_cnt < 11)
//	{
//		return IMU_Error;
//	}
//	else
//	{
//		CRC16 = __CRC16(imu_buffer,9);
//		if(CRC16 != ((imu_buffer[9] << 8) | (imu_buffer[10])))
//		{
//			rx_cnt=0;
//			return IMU_Error;
//		}
//		imu.Euler.roll = (int16_t)((int16_t)imu_buffer[3] << 8 | imu_buffer[4]);
//		imu.Euler.pitch = (int16_t)((int16_t)imu_buffer[5] << 8 | imu_buffer[6]);
//		imu.Euler.yaw = (int16_t)((int16_t)imu_buffer[7] << 8 | imu_buffer[8]);

//		imu.Euler.roll = imu.Euler.roll / 32768.0f * 180.0f;
//		imu.Euler.pitch = imu.Euler.pitch / 32768.0f * 180.0f;
//		imu.Euler.yaw = imu.Euler.yaw / 32768.0f * 180.0f;
//		
//		rx_cnt=0;
//		return IMU_OK;
//	}
//}



/******************************************************************************************/
/* �������´���, ֧��printf����, ������Ҫѡ��use MicroLIB */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* ʹ��AC6������ʱ */
__asm(".global __use_no_semihosting\n\t");          /* ������ʹ�ð�����ģʽ */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6����Ҫ����main����Ϊ�޲�����ʽ�����򲿷����̿��ܳ��ְ�����ģʽ */

#else
/* ʹ��AC5������ʱ, Ҫ�����ﶨ��__FILE �� ��ʹ�ð�����ģʽ */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* ��ʹ�ð�����ģʽ��������Ҫ�ض���_ttywrch\_sys_exit\_sys_command_string����,��ͬʱ����AC6��AC5ģʽ */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* ����_sys_exit()�Ա���ʹ�ð�����ģʽ */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE �� stdio.h���涨��. */
FILE __stdout;

/* �ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
int fputc(int ch, FILE *f)
{
    while ((USART2->SR & 0X40) == 0);             /* �ȴ���һ���ַ�������� */

    USART2->DR = (uint8_t)ch;                     /* ��Ҫ���͵��ַ� ch д�뵽DR�Ĵ��� */
    return ch;
}
#endif
/***********************************************END*******************************************/
