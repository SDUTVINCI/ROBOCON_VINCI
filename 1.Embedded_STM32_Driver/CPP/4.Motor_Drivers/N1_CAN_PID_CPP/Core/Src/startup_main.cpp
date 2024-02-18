#include "startup_main.h"
#include "bsp_delay.h"
#include "can_receive.h"
#include "pid_user.h"

void startup_main(void)
{
//滴答定时器初始化
	bsp_delay.f4.Init(168);
	
//PID初始化
	pid_controller.All_Device_Init();
	
//CAN通信滤波初始化
	can_bus.bsp.Filter_Init(&hcan1);
	can_bus.bsp.Filter_Init(&hcan2);
	
//CAN通信开启
	can_bus.bsp.CAN_Start(&hcan1);
	can_bus.bsp.CAN_Start(&hcan2);


	
	
#if isRTOS==0    	//如果是裸机开发
	for(;;)  //等同于while(true)
	{
		
		//本次程序采用在定时器中断里进行控制电机，而非主函数，请看定时器中断回调函数里的。
		//下方写的是等同于定时器中断中的内容实现。
		
		
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
		
		//尽量维持在1-10ms内，特别是控制角度的时候，计算频率不宜过大。
		bsp_delay.f4.ms(1);
	}
#endif
}