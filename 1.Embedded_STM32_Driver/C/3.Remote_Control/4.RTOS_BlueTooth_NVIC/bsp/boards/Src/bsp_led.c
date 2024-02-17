#include "bsp_led.h"


void green_led_task(void const * argument)
{
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
		osDelay(500);
	}
}
