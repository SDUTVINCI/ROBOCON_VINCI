
#include "data_pack.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
void SendByte2Mapan(uint8_t buffer)
{
	HAL_UART_Transmit(&huart4, &buffer, 1, 1000);
}

void SendByte2Actuator(uint8_t buffer)
{
	HAL_UART_Transmit(&huart2, &buffer, 1, 1000);
}
void Send2Mapan(const uint8_t *data, uint8_t len)
{
	uint8_t i;
	for (i = 0; i < len; i++)
	{
		SendByte2Mapan(data[i]); // 发送一个字节
	}
}
void Send2Actuator(const uint8_t *data, uint8_t len)
{
	uint8_t i;
	for (i = 0; i < len; i++)
	{
		SendByte2Actuator(data[i]); // 发送一个字节
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
void Send_Cmd_Data2Actuator(uint8_t cmd, const uint8_t *datas, uint8_t len)
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
	Send2Actuator(buf, cnt); // 调用数据帧发送函数将打包好的数据帧发送出去
}
void Send_Cmd_Data2Mapan(uint8_t cmd, const uint8_t *datas, uint8_t len)
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
	Send2Mapan(buf, cnt); // 调用数据帧发送函数将打包好的数据帧发送出去
}

uint8_t cmd_myfromActuator;
uint8_t rx_datafromActuator[10];
void Data_AnalysisfromActuator(uint8_t cmd, const uint8_t *datas, uint8_t len)
{

	cmd_myfromActuator = cmd;
	for (int i = 0; i < len; i++)
	{

		rx_datafromActuator[i] = datas[i];
	} // 根据需要处理数据
}

uint8_t rx_datafromMapan[10];
fp32 yaw = 0;
fp32 x = 0;
fp32 y = 0;
extern fp64 yaw_offset;

fp32 omega_speed;
 fp32 now_omega,last_omega;
 extern fp32 yaw;
fp32 Vomega;fp32 yaw_real=0.0;
extern fp64 yaw_offset;
void Data_AnalysisfromMapan(uint8_t cmd, const uint8_t *datas, uint8_t len)
{
	memcpy(&yaw_real, &datas[0], 4);
	
	memcpy(&x, &datas[4], 4);
	memcpy(&y, &datas[8], 4);
	memcpy(&Vomega, &datas[12], 4);
yaw=(fp32)yaw_offset+yaw_real;
	
		
		
		

}
// 接收数据
void ReceivefromMapan(uint8_t bytedata)
{
	static uint8_t stepfromMapan = 0; // 状态变量初始化为0 在函数中必须为静态变量
	static uint8_t cntfromMapan = 0, BuffromMapan[300], lenfromMapan, cmdfromMapan, *data_ptrfromMapan;
	static uint16_t crc16fromMapan;
	// 进行数据解析 状态机
	switch (stepfromMapan)
	{
	case 0: // 接收帧头1状态
		if (bytedata == 0xA5)
		{
			stepfromMapan++;
			cntfromMapan = 0;
			BuffromMapan[cntfromMapan++] = bytedata;
		}
		break;
	case 1: // 接收帧头2状态
		if (bytedata == 0x5A)
		{
			stepfromMapan++;
			BuffromMapan[cntfromMapan++] = bytedata;
		}
		else if (bytedata == 0xA5)
		{
			stepfromMapan = 1;
		}
		else
		{
			stepfromMapan = 0;
		}
		break;
	case 2: // 接收数据长度字节状态
		stepfromMapan++;
		BuffromMapan[cntfromMapan++] = bytedata;
		lenfromMapan = bytedata;
		break;
	case 3: // 接收命令字节状态
		stepfromMapan++;
		BuffromMapan[cntfromMapan++] = bytedata;
		cmdfromMapan = bytedata;
		data_ptrfromMapan = &BuffromMapan[cntfromMapan]; // 记录数据指针首地址
		if (lenfromMapan == 0)
			stepfromMapan++; // 数据字节长度为0则跳过数据接收状态
		break;
	case 4: // 接收len字节数据状态
		BuffromMapan[cntfromMapan++] = bytedata;
		if (data_ptrfromMapan + lenfromMapan == &BuffromMapan[cntfromMapan]) // 利用指针地址偏移判断是否接收完len位数据
		{
			stepfromMapan++;
		}
		break;
	case 5: // 接收crc16校验高8位字节
		stepfromMapan++;
		crc16fromMapan = bytedata;
		break;
	case 6: // 接收crc16校验低8位字节
		crc16fromMapan <<= 8;
		crc16fromMapan += bytedata;
		if (crc16fromMapan == CRC16_Check(BuffromMapan, cntfromMapan)) // 校验正确进入下一状态
		{
			stepfromMapan++;
		}
		else if (bytedata == 0xA5)
		{
			stepfromMapan = 1;
		}
		else
		{
			stepfromMapan = 0;
		}
		break;
	case 7:		      // 接收帧尾
		if (bytedata == 0xFF) // 帧尾接收正确
		{
			Data_AnalysisfromMapan(cmdfromMapan, data_ptrfromMapan, lenfromMapan); // 数据解析
			stepfromMapan = 0;
		}
		else if (bytedata == 0xA5)
		{
			stepfromMapan = 1;
		}
		else
		{
			stepfromMapan = 0;
		}
		break;
	default:
		stepfromMapan = 0;
		break; // 多余状态，正常情况下不可能出现
	}
}

void ReceivefromActuator(uint8_t bytedata)
{
	static uint8_t stepfromActuator = 0; // 状态变量初始化为0 在函数中必须为静态变量
	static uint8_t cntfromActuator = 0, BuffromActuator[300], lenfromActuator, cmdfromActuator, *data_ptrfromActuator;
	static uint16_t crc16fromActuator;
	// 进行数据解析 状态机
	switch (stepfromActuator)
	{
	case 0: // 接收帧头1状态
		if (bytedata == 0xA5)
		{
			stepfromActuator++;
			cntfromActuator = 0;
			BuffromActuator[cntfromActuator++] = bytedata;
		}
		break;
	case 1: // 接收帧头2状态
		if (bytedata == 0x5A)
		{
			stepfromActuator++;
			BuffromActuator[cntfromActuator++] = bytedata;
		}
		else if (bytedata == 0xA5)
		{
			stepfromActuator = 1;
		}
		else
		{
			stepfromActuator = 0;
		}
		break;
	case 2: // 接收数据长度字节状态
		stepfromActuator++;
		BuffromActuator[cntfromActuator++] = bytedata;
		lenfromActuator = bytedata;
		break;
	case 3: // 接收命令字节状态
		stepfromActuator++;
		BuffromActuator[cntfromActuator++] = bytedata;
		cmdfromActuator = bytedata;
		data_ptrfromActuator = &BuffromActuator[cntfromActuator]; // 记录数据指针首地址
		if (lenfromActuator == 0)
			stepfromActuator++; // 数据字节长度为0则跳过数据接收状态
		break;
	case 4: // 接收len字节数据状态
		BuffromActuator[cntfromActuator++] = bytedata;
		if (data_ptrfromActuator + lenfromActuator == &BuffromActuator[cntfromActuator]) // 利用指针地址偏移判断是否接收完len位数据
		{
			stepfromActuator++;
		}
		break;
	case 5: // 接收crc16校验高8位字节
		stepfromActuator++;
		crc16fromActuator = bytedata;
		break;
	case 6: // 接收crc16校验低8位字节
		crc16fromActuator <<= 8;
		crc16fromActuator += bytedata;
		if (crc16fromActuator == CRC16_Check(BuffromActuator, cntfromActuator)) // 校验正确进入下一状态
		{
			stepfromActuator++;
		}
		else if (bytedata == 0xA5)
		{
			stepfromActuator = 1;
		}
		else
		{
			stepfromActuator = 0;
		}
		break;
	case 7:		      // 接收帧尾
		if (bytedata == 0xFF) // 帧尾接收正确
		{
			Data_AnalysisfromActuator(cmdfromActuator, data_ptrfromActuator, lenfromActuator); // 数据解析
			stepfromActuator = 0;
		}
		else if (bytedata == 0xA5)
		{
			stepfromActuator = 1;
		}
		else
		{
			stepfromActuator = 0;
		}
		break;
	default:
		stepfromActuator = 0;
		break; // 多余状态，正常情况下不可能出现
	}
}
