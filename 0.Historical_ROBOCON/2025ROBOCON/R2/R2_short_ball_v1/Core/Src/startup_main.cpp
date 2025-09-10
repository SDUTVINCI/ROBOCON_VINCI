#include "startup_main.h"
#include "N5065_speed.h"
#include "pid_user.h"


extern uint8_t rx_buffer[1];
extern pid_type_def pid_v_1[2];
extern motor_measure N5065[2];
void startup_main(void)
{
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
	Motor_PWM_Init(&htim2);
	HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
	pid_controller.All_Device_Init();
	HAL_TIM_Base_Start_IT(&htim9);
	
#if isRTOS==0    	//如果是裸机开发
	for(;;)  //等同于while(true)
	{
		//printf("%f,%f,%f,%f,%f\n",N5065[0].speed_filtered,pid_v_1[1].out,pid_v_1[1].Pout,pid_v_1[1].Iout,pid_v_1[1].Dout);//pid_v_1[1].out
		 //printf("%f,%f\n",N5065[0].speed_filtered,N5065[1].speed_filtered);//pid_v_1[1].out
		 
		HAL_Delay(1);
	}
#endif
}