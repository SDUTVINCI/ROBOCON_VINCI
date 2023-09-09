#include "pneumatic_task.h"

extern RC_ctrl_t rc_ctrl;
extern uint8_t bluetooth_rx[1];

void pneumatic_task(void const * argument)
{
	vTaskSuspend(NULL);
	while(1)
	{
		if(rc_ctrl.rc.s[0] == 3 || rc_ctrl.rc.s[0] == 1)
		{
			osDelay(300);
			HAL_GPIO_WritePin(Shove_GPIO_Port,Shove_Pin,GPIO_PIN_SET);
			osDelay(300);
			HAL_GPIO_WritePin(Shove_GPIO_Port,Shove_Pin,GPIO_PIN_RESET);
			osDelay(300);
			vTaskSuspend(NULL);
		}
	}
}
