#include "imu_ops9.h"
#include "main.h"
#include "usart.h"
//#include "dma.h"
#include "cmsis_os.h"

//void ops_reset_task(void const * argument)
//{
//	while(1)
//	{
//		
//	}
//}



ops9_t ops9;

uint8_t ops9_buffer[1];



//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == UART5)
//	{
//		//Data_Analyse(ops9_buffer[0]);//dma1读取码盘数据  读取数据在dma中断中
//	}
////	else if(huart->Instance == USART2)
////	{
////		
////	}
////	else if(huart->Instance == UART5)
////	{
////		HAL_UART_Transmit(&huart5,(uint8_t *)(shoot_speed_final),1,1);
////	}
//}

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

//字符串拼接
void stract(char str1[],char str2[],int num)
{
	int i= 0,j = 0;
	while(str1[i] != '\0') i++;
	for(j=0;j<num;j++)
		str1[i++] = str2[j];
}



void Update_x(UART_HandleTypeDef *huart,float new_x)
{
	char update_x[8] = "ACTX";
	static union
	{
		float x;
		char data[4];
	}new_set;
	new_set.x = new_x;
	stract(update_x,new_set.data,4);
	
	HAL_UART_Transmit(huart,(uint8_t *)&update_x,8,portMAX_DELAY);
}


void Update_y(UART_HandleTypeDef *huart,float new_y)
{
	char update_y[8] = "ACTY";
	static union
	{
		float y;
		char data[4];
	}new_set;
	new_set.y = new_y;
	stract(update_y,new_set.data,4);
	
	HAL_UART_Transmit(huart,(uint8_t *)&update_y,8,portMAX_DELAY);
}

void Update_yaw(UART_HandleTypeDef *huart,float new_yaw)
{
	char update_yaw[8] = "ACTJ";
	static union
	{
		float yaw;
		char data[4];
	}new_set;
	new_set.yaw = new_yaw;
	stract(update_yaw,new_set.data,4);
	
	HAL_UART_Transmit(huart,(uint8_t *)&update_yaw,8,portMAX_DELAY);
}
