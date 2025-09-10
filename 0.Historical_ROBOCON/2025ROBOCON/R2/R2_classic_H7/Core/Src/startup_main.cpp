#include "startup_main.h"
#include "fdcan.h"
#include "fdcan_receive.h"
#include "pid.h"
#include "pid_user.h"
#include "usart.h"
#include "Communition.h"
extern Rxbuffer all_rxbuff;
void startup_main(void)
{	

  HAL_UART_Receive_DMA(&huart5, all_rxbuff.DR16, 18);

#if isRTOS==1    	//如果是裸机开发
	for(;;)  //等同于while(true)
	{
		// VESC_CAN_Speedrpm(0x0311,1000);
//		HAL_Delay(100);
		// can_bus.cmd.FDCAN1_Front(0,
		// 	0 ,
		// 	0,
		// 	pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize(-9000, 4,PID_RELATIVE_POS));
//发射炮台角度量程11万
	}
#endif
}

