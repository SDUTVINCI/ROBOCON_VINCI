#include "led_task.h"

// __weakÈõº¯Êý
extern "C"
void led_task(void const * argument)
{
	while(true)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		osDelay(500);
	}
}

