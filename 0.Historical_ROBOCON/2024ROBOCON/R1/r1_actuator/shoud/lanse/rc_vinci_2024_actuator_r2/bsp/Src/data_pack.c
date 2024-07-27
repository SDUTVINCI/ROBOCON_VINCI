
#include "data_pack.h"
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;


void SendByte2chassis(uint8_t buffer)
{
	HAL_UART_Transmit(&huart2, &buffer, 1, 1000);
}
void Send2chassis(const uint8_t *data, uint8_t len)
{
	uint8_t i;
	for (i = 0; i < len; i++)
	{
		SendByte2chassis(data[i]); // ����һ���ֽ�
	}
}
void SendByte2Gimbal(uint8_t buffer)
{
	HAL_UART_Transmit(&huart6, &buffer, 1, 1000);
}
void Send2Gimbal(const uint8_t *data, uint8_t len)
{
	uint8_t i;
	for (i = 0; i < len; i++)
	{
		SendByte2Gimbal(data[i]); // ����һ���ֽ�
	}
}
uint16_t CRC16_Check(const uint8_t *data, uint8_t len)
{
	uint16_t CRC16 = 0xFFFF;
	uint8_t state, i, j;
	for (i = 0; i < len; i++)
	{
		CRC16 ^= data[i];
		for (j = 0; j < 8; j++)
		{
			state = CRC16 & 0x01;
			CRC16 >>= 1;
			if (state)
			{
				CRC16 ^= 0xA001;
			}
		}
	}
	return CRC16;
}
void Send_Cmd_Data2chassis(uint8_t cmd, const uint8_t *datas, uint8_t len)
{
	uint8_t buf[300], i, cnt = 0;
	uint16_t crc16;
	buf[cnt++] = 0xA5;
	buf[cnt++] = 0x5A;
	buf[cnt++] = len;
	buf[cnt++] = cmd;
	for (i = 0; i < len; i++)
	{
		buf[cnt++] = datas[i];
	}
	crc16 = CRC16_Check(buf, len + 4);
	buf[cnt++] = crc16 >> 8;
	buf[cnt++] = crc16 & 0xFF;
	buf[cnt++] = 0xFF;
	Send2chassis(buf, cnt); // ��������֡���ͺ���������õ�����֡���ͳ�ȥ
}
void Send_Cmd_Data2Gimbal(uint8_t cmd, const uint8_t *datas, uint8_t len)
{
	uint8_t buf[300], i, cnt = 0;
	uint16_t crc16;
	buf[cnt++] = 0xA5;
	buf[cnt++] = 0x5A;
	buf[cnt++] = len;
	buf[cnt++] = cmd;
	for (i = 0; i < len; i++)
	{
		buf[cnt++] = datas[i];
	}
	crc16 = CRC16_Check(buf, len + 4);
	buf[cnt++] = crc16 >> 8;
	buf[cnt++] = crc16 & 0xFF;
	buf[cnt++] = 0xFF;
	Send2Gimbal(buf, cnt); // ��������֡���ͺ���������õ�����֡���ͳ�ȥ
}





uint8_t cmd_myfromChassis;
uint8_t rx_datafromChassis[10];
void Data_AnalysisfromChassis(uint8_t cmd, const uint8_t *datas, uint8_t len)
{

	cmd_myfromChassis = cmd;
	for (int i = 0; i < len; i++)
	{

		rx_datafromChassis[i] = datas[i];
	} // ������Ҫ��������
}

// ��������
void ReceivefromChassis(uint8_t bytedata)
{
	static uint8_t stepfromChassis = 0; // ״̬������ʼ��Ϊ0 �ں����б���Ϊ��̬����
	static uint8_t cntfromChassis = 0, BuffromChassis[40], lenfromChassis, cmdfromChassis, *data_ptrfromChassis;
	static uint16_t crc16fromChassis;
	// �������ݽ��� ״̬��
	switch (stepfromChassis)
	{
	case 0: // ����֡ͷ1״̬
		if (bytedata == 0xA5)
		{
			stepfromChassis++;
			cntfromChassis = 0;
			BuffromChassis[cntfromChassis++] = bytedata;
		}
		break;
	case 1: // ����֡ͷ2״̬
		if (bytedata == 0x5A)
		{
			stepfromChassis++;
			BuffromChassis[cntfromChassis++] = bytedata;
		}
		else if (bytedata == 0xA5)
		{
			stepfromChassis = 1;
		}
		else
		{
			stepfromChassis = 0;
		}
		break;
	case 2: // �������ݳ����ֽ�״̬
		stepfromChassis++;
		BuffromChassis[cntfromChassis++] = bytedata;
		lenfromChassis = bytedata;
		break;
	case 3: // ���������ֽ�״̬
		stepfromChassis++;
		BuffromChassis[cntfromChassis++] = bytedata;
		cmdfromChassis = bytedata;
		data_ptrfromChassis = &BuffromChassis[cntfromChassis]; // ��¼����ָ���׵�ַ
		if (lenfromChassis == 0)
			stepfromChassis++; // �����ֽڳ���Ϊ0���������ݽ���״̬
		break;
	case 4: // ����len�ֽ�����״̬
		BuffromChassis[cntfromChassis++] = bytedata;
		if (data_ptrfromChassis + lenfromChassis == &BuffromChassis[cntfromChassis]) // ����ָ���ַƫ���ж��Ƿ������lenλ����
		{
			stepfromChassis ++ ;
		}
		break;
	case 5: // ����crc16У���8λ�ֽ�
		stepfromChassis++;
		crc16fromChassis = bytedata;
		break;
	case 6: // ����crc16У���8λ�ֽ�
		crc16fromChassis <<= 8;
		crc16fromChassis += bytedata;
		if (crc16fromChassis == CRC16_Check(BuffromChassis, cntfromChassis)) // У����ȷ������һ״̬
		{
			stepfromChassis++;
		}
		else if (bytedata == 0xA5)
		{
			stepfromChassis = 1;
		}
		else
		{
			stepfromChassis = 0;
		}
		break;
	case 7:		      // ����֡β
		if (bytedata == 0xFF) // ֡β������ȷ
		{
			Data_AnalysisfromChassis(cmdfromChassis, data_ptrfromChassis, lenfromChassis); // ���ݽ���
			stepfromChassis = 0;
		}
		else if (bytedata == 0xA5)
		{
			stepfromChassis = 1;
		}
		else
		{
			stepfromChassis = 0;
		}
		break;
	default:
		stepfromChassis = 0;
		break; // ����״̬����������²����ܳ���
	}
}

