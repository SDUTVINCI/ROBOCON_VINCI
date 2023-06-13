#include "ops9.h"

IMU_t ops9;

uint8_t ops9_buffer[1];


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		Data_Analyse(ops9_buffer[0]);
	}
}

/**
 * @brief ���ݽ�������  �����MCUƽ̨���������⣬ֻ�轫���ڽ��յ���ֵ����ú������ɽ���
 * @param  rec ���ڽ��յ����ֽ�����
 */
void Data_Analyse(uint8_t rec)
{
	static uint8_t ch;
	static union
	{
		uint8_t date[24];
		fp32 ActVal[6];
	}posture;
	static uint8_t count=0;
	static uint8_t i=0;

	ch=rec;
	switch(count)
	{
		case 0:
			if(ch==0x0d)
				count++;
			else
				count=0;
			break;
		case 1:
			if(ch==0x0a)
			{
				i=0;
				count++;
			}
			else if(ch==0x0d);
			else
				count=0;
			break;
		case 2:
			posture.date[i]=ch;
			i++;
			if(i>=24)
			{
				i=0;
				count++;
			}
			break;
		case 3:
			if(ch==0x0a)
				count++;
			else
				count=0;
			break;
		case 4:
			if(ch==0x0d)
			{
				ops9.Euler.yaw=posture.ActVal[0];   		//zangle
				ops9.Euler.pitch=posture.ActVal[1];			//xangle
				ops9.Euler.roll=posture.ActVal[2];			//yangle
				ops9.Position.pos_x=posture.ActVal[3];
				ops9.Position.pos_y=posture.ActVal[4];
				ops9.Euler_velocity.yaw_velocity=posture.ActVal[5];
			}
			count=0;
			break;
		default:
			count=0;
		break;
	}
}

//�ַ���ƴ��
void stract(char str1[],char str2[],int num)
{
	int i= 0,j = 0;
	while(str1[i] != '\0') i++;
	for(j=0;j<num;j++)
		str1[i++] = str2[j];
}



void Update_x(float new_x)
{
	int i=0;
	char update_x[8] = "ACTX";
	static union
	{
		float x;
		char data[4];
	}new_set;
	new_set.x = new_x;
	stract(update_x,new_set.data,4);
	for(i=0;i<8;i++)
	{
		HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&update_x,1);
//		while(huart6.Instance->SR | UART_FLAG_TXE);
//		USART_SendData(USART6, update_x);	
	}
}


void Update_y(float new_y)
{
	int i=0;
	char update_y[8] = "ACTY";
	static union
	{
		float y;
		char data[4];
	}new_set;
	new_set.y = new_y;
	stract(update_y,new_set.data,4);
	for(i=0;i<8;i++)
	{
//		while(huart6.Instance->SR | UART_FLAG_TXE);
//		USART_SendData(USART6, update_y);		
		HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&update_y,1);
	}
}

void Update_yaw(float new_yaw)
{
	int i=0;
	char update_yaw[8] = "ACTJ";
	static union
	{
		float yaw;
		char data[4];
	}new_set;
	new_set.yaw = new_yaw;
	stract(update_yaw,new_set.data,4);
	for(i=0;i<8;i++)
	{
//		while(huart6.Instance->SR | UART_FLAG_TXE);
//		USART_SendData(USART6, update_yaw);		
		HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&update_yaw,1);
	}
}

