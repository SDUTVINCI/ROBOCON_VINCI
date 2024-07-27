
#include "data_pack.h"
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

void SendByte2pc(uint8_t buffer)
{
	HAL_UART_Transmit(&huart3, &buffer, 1, 1000);
}
void Send2pc(const uint8_t *data, uint8_t len)
{
	uint8_t i;
	for (i = 0; i < len; i++)
	{
		SendByte2pc(data[i]); // ����һ���ֽ�
	}
}
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

void Send_Cmd_Data2pc(uint8_t cmd, const uint8_t *datas, uint8_t len)
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
	Send2pc(buf, cnt); // ��������֡���ͺ���������õ�����֡���ͳ�ȥ
}
extern int16_t dx;
extern int16_t distance_carmere;
extern uint8_t resume_track_ball;
uint8_t cmd_myfromPc = 0x96;
uint8_t rx_datafromPc[10];
extern uint8_t if_stop_target;
void Data_AnalysisfromPc(uint8_t cmd, const uint8_t *datas, uint8_t len)
{

	cmd_myfromPc = cmd;
	for (int i = 0; i < len; i++)
	{

		rx_datafromPc[i] = datas[i];

	} // ������Ҫ��������
	if (cmd_myfromPc == 0x07)
	{

		memcpy(&distance_carmere, &rx_datafromPc[0], 2);
		memcpy(&dx, &rx_datafromPc[2], 2);
		// resume_track_ball=1;
		if_stop_target = 0;
	}

	else if (cmd_myfromPc == 0x06)
	{
		if (rx_datafromPc[0] == 0x01)
		{
			if_stop_target = 1;
		}
	}
}
extern uint8_t task_step;
uint8_t cmd_myfromChassis;
uint8_t rx_datafromChassis[10];
extern int motor_speed[3];
void Data_AnalysisfromChassis(uint8_t cmd, const uint8_t *datas, uint8_t len)
{
	// if (cmd == 0x48)
	// {
	// 	switch (datas[0])
	// 	{
	// 		//		case 0x00: // ���0���������  ִ��1������//λ��У׼���
	// 		//			task_step = 1;
	// 		//			break;
	// 		//			// 	case 0x01: // ���1���������  ִ��2������
	// 		//			// 		task_step = 2;
	// 		//			// 		break;
	// 		//			// 	case 0x02: // ���2���������  ִ��0������
	// 		//			// 		task_step = 0;
	// 		//			// 		break;
	// 		//		case 0x03:
	// 		//			motor_speed[0] = 1000;
	// 		//			motor_speed[1] = 1000;
	// 		//			break;
	// 		//		case 0x04:
	// 		//			motor_speed[0] = 0;
	// 		//			motor_speed[1] = 0;
	// 		//			task_step = 0;
	// 		//			break;
	// 	default:
	// 		break;
	// 	}
	// }

	for (int i = 0; i < len; i++)
	{
		cmd_myfromChassis = cmd;
		rx_datafromChassis[i] = datas[i];
	} // ������Ҫ��������
}

// ��������
void ReceivefromChassis(uint8_t bytedata)
{
	static uint8_t stepfromChassis = 0; // ״̬������ʼ��Ϊ0 �ں����б���Ϊ��̬����
	static uint8_t cntfromChassis = 0, BuffromChassis[300], lenfromChassis, cmdfromChassis, *data_ptrfromChassis;
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
			stepfromChassis++;
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

// ��������
void ReceivefromPc(uint8_t bytedata)
{
	static uint8_t stepfromPc = 0; // ״̬������ʼ��Ϊ0 �ں����б���Ϊ��̬����
	static uint8_t cntfromPc = 0, BuffromPc[300], lenfromPc, cmdfromPc, *data_ptrfromPc;
	static uint16_t crc16fromPc;
	// �������ݽ��� ״̬��
	switch (stepfromPc)
	{
	case 0: // ����֡ͷ1״̬
		if (bytedata == 0xA5)
		{
			stepfromPc++;
			cntfromPc = 0;
			BuffromPc[cntfromPc++] = bytedata;
		}
		break;
	case 1: // ����֡ͷ2״̬
		if (bytedata == 0x5A)
		{
			stepfromPc++;
			BuffromPc[cntfromPc++] = bytedata;
		}
		else if (bytedata == 0xA5)
		{
			stepfromPc = 1;
		}
		else
		{
			stepfromPc = 0;
		}
		break;
	case 2: // �������ݳ����ֽ�״̬
		stepfromPc++;
		BuffromPc[cntfromPc++] = bytedata;
		lenfromPc = bytedata;
		break;
	case 3: // ���������ֽ�״̬
		stepfromPc++;
		BuffromPc[cntfromPc++] = bytedata;
		cmdfromPc = bytedata;
		data_ptrfromPc = &BuffromPc[cntfromPc]; // ��¼����ָ���׵�ַ
		if (lenfromPc == 0)
			stepfromPc++; // �����ֽڳ���Ϊ0���������ݽ���״̬
		break;
	case 4: // ����len�ֽ�����״̬
		BuffromPc[cntfromPc++] = bytedata;
		if (data_ptrfromPc + lenfromPc == &BuffromPc[cntfromPc]) // ����ָ���ַƫ���ж��Ƿ������lenλ����
		{
			stepfromPc++;
		}
		break;
	case 5: // ����crc16У���8λ�ֽ�
		stepfromPc++;
		crc16fromPc = bytedata;
		break;
	case 6: // ����crc16У���8λ�ֽ�
		crc16fromPc <<= 8;
		crc16fromPc += bytedata;
		if (crc16fromPc == CRC16_Check(BuffromPc, cntfromPc)) // У����ȷ������һ״̬
		{
			stepfromPc++;
		}
		else if (bytedata == 0xA5)
		{
			stepfromPc = 1;
		}
		else
		{
			stepfromPc = 0;
		}
		break;
	case 7:		      // ����֡β
		if (bytedata == 0xFF) // ֡β������ȷ
		{
			Data_AnalysisfromPc(cmdfromPc, data_ptrfromPc, lenfromPc); // ���ݽ���
			stepfromPc = 0;
		}
		else if (bytedata == 0xA5)
		{
			stepfromPc = 1;
		}
		else
		{
			stepfromPc = 0;
		}
		break;
	default:
		stepfromPc = 0;
		break; // ����״̬����������²����ܳ���
	}
}

// �����������ݲ��ȴ���λ������Ӧ��
void SendPakageWaitACK(int Device, uint8_t Cmd, uint8_t *SendData, uint8_t SendDataLen, uint8_t AckData)
{
	if (Device == Chassis)
	{
		cmd_myfromChassis = 0xff;
		while (!((rx_datafromChassis[0] == AckData) && (cmd_myfromChassis == Cmd)))
		{
			cmd_myfromChassis = 0xff;
			Send_Cmd_Data2chassis(Cmd, SendData, SendDataLen);
			osDelay(10);
		}
		cmd_myfromChassis = 0xff;
	}
	else if (Device == Pc)
	{
		cmd_myfromPc = 0xff;
		while (!((rx_datafromPc[0] == AckData) && (cmd_myfromPc == Cmd)))
		{
			cmd_myfromPc = 0xff;
			Send_Cmd_Data2pc(Cmd, SendData, SendDataLen);
			osDelay(10);
		}
		cmd_myfromPc = 0xff;
	}
}

void WaitPakageSendACK(int Device, uint8_t Cmd, uint8_t *ReciveData, uint8_t DataLen, uint8_t AckData)
{

	if (Device == Chassis)
	{
		while (!((rx_datafromChassis[0] == ReciveData[0]) && (cmd_myfromChassis == Cmd)))
		{
			osDelay(5);
		}
		while ((rx_datafromChassis[0] == ReciveData[0]) && (cmd_myfromChassis == Cmd))
		{
			cmd_myfromChassis = 0xff;
			Send_Cmd_Data2chassis(Cmd, &AckData, 1);

			osDelay(20);
		}
		cmd_myfromChassis = 0xff;
	}
	else if (Device == Pc)
	{
		while (!((rx_datafromPc[0] == ReciveData[0]) && (cmd_myfromPc == Cmd)))
		{
			osDelay(5);
		}
		while ((rx_datafromPc[0] == ReciveData[0]) && (cmd_myfromPc == Cmd))
		{
			cmd_myfromPc = 0xff;
			Send_Cmd_Data2pc(Cmd, &AckData, 1);

			osDelay(20);
		}
		cmd_myfromPc = 0xff;
	}
}
