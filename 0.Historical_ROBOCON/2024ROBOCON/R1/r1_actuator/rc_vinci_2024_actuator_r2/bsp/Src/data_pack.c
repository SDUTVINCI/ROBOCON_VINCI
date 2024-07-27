
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
		SendByte2chassis(data[i]); // 发送一个字节
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
		SendByte2Gimbal(data[i]); // 发送一个字节
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
	Send2chassis(buf, cnt); // 调用数据帧发送函数将打包好的数据帧发送出去
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
	Send2Gimbal(buf, cnt); // 调用数据帧发送函数将打包好的数据帧发送出去
}





uint8_t cmd_myfromChassis;
uint8_t rx_datafromChassis[10];
void Data_AnalysisfromChassis(uint8_t cmd, const uint8_t *datas, uint8_t len)
{

	cmd_myfromChassis = cmd;
	for (int i = 0; i < len; i++)
	{

		rx_datafromChassis[i] = datas[i];
	} // 根据需要处理数据
}

// 接收数据
void ReceivefromChassis(uint8_t bytedata)
{
	static uint8_t stepfromChassis = 0; // 状态变量初始化为0 在函数中必须为静态变量
	static uint8_t cntfromChassis = 0, BuffromChassis[40], lenfromChassis, cmdfromChassis, *data_ptrfromChassis;
	static uint16_t crc16fromChassis;
	// 进行数据解析 状态机
	switch (stepfromChassis)
	{
	case 0: // 接收帧头1状态
		if (bytedata == 0xA5)
		{
			stepfromChassis++;
			cntfromChassis = 0;
			BuffromChassis[cntfromChassis++] = bytedata;
		}
		break;
	case 1: // 接收帧头2状态
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
	case 2: // 接收数据长度字节状态
		stepfromChassis++;
		BuffromChassis[cntfromChassis++] = bytedata;
		lenfromChassis = bytedata;
		break;
	case 3: // 接收命令字节状态
		stepfromChassis++;
		BuffromChassis[cntfromChassis++] = bytedata;
		cmdfromChassis = bytedata;
		data_ptrfromChassis = &BuffromChassis[cntfromChassis]; // 记录数据指针首地址
		if (lenfromChassis == 0)
			stepfromChassis++; // 数据字节长度为0则跳过数据接收状态
		break;
	case 4: // 接收len字节数据状态
		BuffromChassis[cntfromChassis++] = bytedata;
		if (data_ptrfromChassis + lenfromChassis == &BuffromChassis[cntfromChassis]) // 利用指针地址偏移判断是否接收完len位数据
		{
			stepfromChassis ++ ;
		}
		break;
	case 5: // 接收crc16校验高8位字节
		stepfromChassis++;
		crc16fromChassis = bytedata;
		break;
	case 6: // 接收crc16校验低8位字节
		crc16fromChassis <<= 8;
		crc16fromChassis += bytedata;
		if (crc16fromChassis == CRC16_Check(BuffromChassis, cntfromChassis)) // 校验正确进入下一状态
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
	case 7:		      // 接收帧尾
		if (bytedata == 0xFF) // 帧尾接收正确
		{
			Data_AnalysisfromChassis(cmdfromChassis, data_ptrfromChassis, lenfromChassis); // 数据解析
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
		break; // 多余状态，正常情况下不可能出现
	}
}

