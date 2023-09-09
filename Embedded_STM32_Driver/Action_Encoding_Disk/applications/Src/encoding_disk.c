#include "encoding_disk.h"
#include "usart.h"



extern uint8_t Res_buffer;


void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
{
	USARTx->DR = (Data & (uint16_t)0x01FF);
}


uint16_t USART_ReceiveData(USART_TypeDef* USARTx)
{
	 return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);
}




/**
 * @brief 解析结果变量，如需跨文件调用，需要外部声明
 */
 
 
 encoding_disk_measure_t gesture;

//void USART1_IRQHandler(void)                	//串口6中断服务程序
//{
//	int8_t Res;
//	if(huart1.Instance->SR & UART_FLAG_RXNE)									//接收中断
//	{
//		__HAL_UART_CLEAR_PEFLAG(&huart1);
//		Res = huart1.Instance->DR;
//	}
////	 else if(huart6.Instance->SR & UART_FLAG_IDLE)          //空闲中断
////  {
////		__HAL_UART_CLEAR_PEFLAG(&huart6);
////    Res = huart6.Instance->DR;
////	}
//	Data_Analyse(Res);
//} 

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance == USART6)
		{
				Data_Analyse(Res_buffer);
		}
}




/**
 * @brief 数据解析函数  如更换MCU平台或更换软件库，只需将串口接收到的值传入该函数即可解析
 * @param  rec 串口接收到的字节数据
 */
void Data_Analyse(uint8_t rec)
{
	static uint8_t ch;
	static union
	{
		uint8_t date[24];
		float ActVal[6];
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
				gesture.zangle=posture.ActVal[0];
				gesture.xangle=posture.ActVal[1];
				gesture.yangle=posture.ActVal[2];
				gesture.pos_x=posture.ActVal[3];
				gesture.pos_y=posture.ActVal[4];
				gesture.w_z=posture.ActVal[5];
			}
			count=0;
			break;
		default:
			count=0;
		break;
	}
}


//字符串拼接
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
//		while(huart6.Instance->SR | UART_FLAG_TXE);
//		USART_SendData(USART6, update_x);	
		HAL_UART_Transmit(&huart6,(uint8_t *)&update_x,1,HAL_MAX_DELAY);
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
		HAL_UART_Transmit(&huart6,(uint8_t *)&update_y,1,HAL_MAX_DELAY);
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
		HAL_UART_Transmit(&huart6,(uint8_t *)&update_yaw,1,HAL_MAX_DELAY);
	}
}


