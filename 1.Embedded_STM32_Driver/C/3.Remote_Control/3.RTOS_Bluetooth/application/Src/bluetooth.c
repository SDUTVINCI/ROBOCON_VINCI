#include "bluetooth.h"

extern osThreadId LED_GREENHandle;
extern osThreadId BLUETOOTHHandle;



void bluetooth_task(void const * argument)
{
	uint8_t RX;
	HAL_UART_Transmit(&huart1,"ready\r\n",7,HAL_MAX_DELAY);   //发送函数 串口号，发送内容，内容长度，HAL_MAX_DELAY \r\n算两个字节
	while(1)
	{
		HAL_UART_Receive(&huart1,&RX,1,HAL_MAX_DELAY);//接收函数，放在while(1)里，让它一直接收, 串口号，存储位置，长度，HAL_MAX_DELAY
		switch(RX)
		{
			case '0':vTaskSuspend(LED_GREENHandle);break;
			case '1':vTaskResume(LED_GREENHandle);break;
			default:;
//			case '0':HAL_UART_Transmit(&huart1,"on\r\n",4,HAL_MAX_DELAY);break;//on 2个字节+\r\n 一共4个字节
//			case '1':HAL_UART_Transmit(&huart1,"off\r\n",5,HAL_MAX_DELAY);break;
//			default:HAL_UART_Transmit(&huart1,"none\r\n",6,HAL_MAX_DELAY);break;
		}
		vTaskDelay(5);
	}

}



