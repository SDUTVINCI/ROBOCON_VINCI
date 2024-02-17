#include "bluetooth.h"

uint8_t rx_buffer[1];
extern osSemaphoreId USART1_BinarySem01Handle;
extern osThreadId LED_GREENHandle;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		xSemaphoreGiveFromISR(USART1_BinarySem01Handle,NULL);
	}
}



void bluetooth_task(void const * argument)           //此任务优先级可以比LED灯高（因为有阻塞），效果会更好，也可以相同。
{
	while(1)
	{
		xSemaphoreTake(USART1_BinarySem01Handle,portMAX_DELAY);
			switch(rx_buffer[0])
		{
			case '0':vTaskSuspend(LED_GREENHandle);break;
			case '1':vTaskResume(LED_GREENHandle);break;
			default:;
		}
	}
}
