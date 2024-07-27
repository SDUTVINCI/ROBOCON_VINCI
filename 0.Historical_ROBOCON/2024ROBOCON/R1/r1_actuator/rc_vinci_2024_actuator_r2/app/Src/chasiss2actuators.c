#include "chasiss2actuators.h"
 
#include <stdio.h>

uint8_t  rx_buffer[1];
 
 

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		 
	}
}