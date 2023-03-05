#include "bluetooth.h"

uint8_t rx_buffer[1];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		switch(rx_buffer[0])
		{
			case '0':HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_RESET);break;
			case '1':HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);break;
			default:;
		}
		
	}
}

