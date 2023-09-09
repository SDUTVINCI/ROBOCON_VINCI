#include "bluetooth.h"

extern osThreadId LED_GREENHandle;
extern osThreadId BLUETOOTHHandle;



void bluetooth_task(void const * argument)
{
	uint8_t RX;
	HAL_UART_Transmit(&huart1,"ready\r\n",7,HAL_MAX_DELAY);   //���ͺ��� ���ںţ��������ݣ����ݳ��ȣ�HAL_MAX_DELAY \r\n�������ֽ�
	while(1)
	{
		HAL_UART_Receive(&huart1,&RX,1,HAL_MAX_DELAY);//���պ���������while(1)�����һֱ����, ���ںţ��洢λ�ã����ȣ�HAL_MAX_DELAY
		switch(RX)
		{
			case '0':vTaskSuspend(LED_GREENHandle);break;
			case '1':vTaskResume(LED_GREENHandle);break;
			default:;
//			case '0':HAL_UART_Transmit(&huart1,"on\r\n",4,HAL_MAX_DELAY);break;//on 2���ֽ�+\r\n һ��4���ֽ�
//			case '1':HAL_UART_Transmit(&huart1,"off\r\n",5,HAL_MAX_DELAY);break;
//			default:HAL_UART_Transmit(&huart1,"none\r\n",6,HAL_MAX_DELAY);break;
		}
		vTaskDelay(5);
	}

}



