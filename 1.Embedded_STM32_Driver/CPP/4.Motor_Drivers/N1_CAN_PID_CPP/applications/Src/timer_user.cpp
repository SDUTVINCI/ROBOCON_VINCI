#include "timer_user.h"
#include "can_receive.h"
#include "pid_user.h"

extern "C"
/**
 * @brief       回调函数，定时器中断服务函数调用
 * @param       htim：触发中断的定时器句柄
 * @retval      void
 * @note        该函数由中断公共服务函数调用，不用用户去调用。且为一个弱函数，所以在C++中要在该函数前面加上extern "C"，或直接用extern "C"{}括起来
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM7)  //周期为1ms
	{
		//开环，使CAN1的1号电机以1500的相对电流值进行转动
		can_bus.cmd.CAN1_Front(1500,0,0,0);
		
		
		/*闭环    数组是以0开始，电调ID以1开始。
		使CAN1数组序号为4(实则电调ID为5)的电机以500rpm的速度转动。
		使CAN1数组序号为5(实则电调ID为6)的电机以600rpm的速度转动。
		使CAN1数组序号为6(实则电调ID为7)的电机转动到4000的相对角度停下。
		使CAN1数组序号为7(实则电调ID为8)的电机开环以0相对电流值驱动。
		*/
		can_bus.cmd.CAN1_Behind(
		pid_controller.can_motor.CAN1_Velocity_Realize(500,4),
		pid_controller.can_motor.CAN1_Velocity_Realize(600,5),
		pid_controller.can_motor.CAN1_Position_Realize(4000,6),
		0);
	}
}
