#include "timer_user.h"
#include "chassic.h"
#include "fdcan_receive.h"
#include "Communition.h"
#include "my_task.h"
#include "pid.h"
#include "pid_user.h"
#include "MT6701.h"
#include "DR16.h"
#include "bsp_usart.h"

extern Rxbuffer all_rxbuff;
extern int16_t N5065_traget_speed[2];
extern bool wake_fireball_task;
extern remote_data receive_data;
 
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
        //炮台角度2500-5850 / -34.66° - 46°
        Get_Remote_Date();
        MT6701_ReadAngle(&hi2c2);
        BlueTooth_Send_Data(4);
//底部气缸手动控制
	if (all_rxbuff.s2 == 1 && all_rxbuff.s1 == 2 ) {
        if (robot.rod_state.bottom == true)  ROD_BOTTOM_PUSH;
        if (robot.rod_state.bottom == false) ROD_BOTTOM_BACK;

        // if (robot.stop == true)  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        // if (robot.stop == false) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    }
if (all_rxbuff.s2 == 1) {
    if (wake_fireball_task == 1) {
        if (robot.upper_data[3] < 1.7) {
            N5065_traget_speed[0] = N5065_traget_speed[1] = 1500;
        }
        else if (robot.upper_data[3] > 3.1) {
            N5065_traget_speed[0] = N5065_traget_speed[1] = robot.upper_data[4] - 200;
        }
        else {
            N5065_traget_speed[0] = N5065_traget_speed[1] = robot.upper_data[4] + 200;
        }
    }
}

		
			if (all_rxbuff.s2 == 1 && all_rxbuff.s1 == 2 ) {
                // if (robot.fixed_point_flag ==0) {
                
                    N5065_traget_speed[0] = N5065_traget_speed[1] = receive_data.ch[0];	
                    // robot.fixed_point_flag = 1;
                // }
			}
			
        VESC_CAN_Speedrpm(0x0311,N5065_traget_speed[0]);
        VESC_CAN_Speedrpm(0x0346,N5065_traget_speed[1]);

        if (all_rxbuff.s2 == MANUAL_MODE){
        can_bus.cmd.FDCAN1_Behind(
            pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize(robot.target_ball_angle, 5,PID_EXTERNAL_POS),
        	0,
        	0,
        	0);
        }
        if (all_rxbuff.s2 != MANUAL_MODE){
            if (all_rxbuff.s2 == 1 && all_rxbuff.s1 == 2) {
                    can_bus.cmd.FDCAN1_Behind(
                    pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize(robot.BT_target_angle, 5,PID_EXTERNAL_POS),
                    0,
                    0,
                    0); 
            }
            else {
                if (robot.Distance <=3.05) {
                    can_bus.cmd.FDCAN1_Behind(
                    pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize(-15, 5,PID_EXTERNAL_POS),
                    0,
                    0,
                    0);
                }
                else {
                    can_bus.cmd.FDCAN1_Behind(
                    pid_controller.can_motor.CAN1_VP_Dual_Loop_Realize(-17, 5,PID_EXTERNAL_POS),
                    0,
                    0,
                    0);
                }
            }
	}
}
}
