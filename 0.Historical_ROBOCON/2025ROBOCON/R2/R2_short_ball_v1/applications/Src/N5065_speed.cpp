#include "N5065_speed.h"
#include "pid_user.h"

motor_measure N5065[2] ={
    {0, 0 ,0,
        0,0,0,
        0,TIM_CHANNEL_1},
    {0, 0 ,0,
        0,0,0,
        0,TIM_CHANNEL_2}
};
uint32_t capture_buffer[2];
uint8_t cap_count = 0;

int32_t Motor_target[2];


/**
 * @brief 定时器更新中断
 * 
 * @param htim 
 */
 extern "C"
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {
    uint32_t speed[2] = {0,0};
     if (htim->Instance == TIM9) {
        Judge_Motor_Stop(&N5065[0]);
        Judge_Motor_Stop(&N5065[1]);
        speed[0] =  pid_controller.can_motor.N5065_Velocity_Realize(Motor_target[0], 1);
        speed[1] =  pid_controller.can_motor.N5065_Velocity_Realize(Motor_target[1], 2);
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,speed[0]+1000);
        __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,speed[1]+1000);
     }
 }

/*
    用来存放定时器输入捕获的相对时间值（在某一次确定的时刻arr的计数值）
    计数频率由周期计算公式决定，time(s) = arr / MHz 
*/
extern "C"
    void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
    {
		if (htim->Instance == TIM3) {
            // switch(htim->Channel)
            // {
            //     case HAL_TIM_ACTIVE_CHANNEL_1:
                  
            //         speed_rpm_calc(&N5065[0]);
            //         break;
            //     case HAL_TIM_ACTIVE_CHANNEL_2:
                 
            //         speed_rpm_calc(&N5065[1]);
			// 					// cap_count++;
            //         break;
            //     default:
            //         break;
            // }
            speed_rpm_calc(&htim3,&N5065[0]);
        }
        if (htim->Instance == TIM4) {
            speed_rpm_calc(&htim4,&N5065[1]);
        }
    }



void speed_rpm_calc(TIM_HandleTypeDef *htim , motor_measure *ptr)
{
    ptr->cap_now = HAL_GetTick(); // 毫秒
    if (ptr->cap_init == 0) {
        ptr->last_cap_val = HAL_TIM_ReadCapturedValue(htim, ptr->Channel);
        ptr->cap_init = 1;
        ptr->speed_filtered = 0;

        return;
    }
   if (ptr->cap_init == 1) {
        
        ptr->cap_val = HAL_TIM_ReadCapturedValue(htim, ptr->Channel);
        
        uint32_t diff_us = (ptr->cap_val >= ptr->last_cap_val)
        ? (ptr->cap_val -  ptr->last_cap_val)
        : (65535 - ptr->last_cap_val + ptr->cap_val + 1); // 处理计数器溢出,最大值
        
        // if (diff_us  == 0 || diff_us > 10000000) return;
        
          fp32 freq = 1000000.0f/diff_us;//频率(多少个上升沿/秒)

        ptr->speed_rpm = freq * (60.0f / 7.0f);//*60 有多少个上升沿/分钟 /7（一圈有七个上升沿也就是一转） = rpm
        ptr->speed_filtered = ALPHA * ptr->speed_rpm + (1.0f - ALPHA) * ptr->speed_filtered;

        ptr->last_cap_val = ptr->cap_val;
        ptr->last_cap_tim = ptr->cap_now;
   }

}
void Judge_Motor_Stop(motor_measure *ptr)
{
    if (HAL_GetTick() - ptr->last_cap_tim > 300)  // 超过300ms无信号
    {
				ptr->speed_rpm = 0 ;
        ptr->speed_filtered = 0;
    }
}

void Motor_PWM_Init(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1 );
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2 );
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_1 ,2000);
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_2 ,2000);
    // HAL_Delay(3000);
    HAL_Delay(3000);
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_1 ,1000);
    __HAL_TIM_SetCompare(htim,TIM_CHANNEL_2 ,1000);
    // HAL_Delay(3000);
    HAL_Delay(3000);
    
//  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,1000);
//  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,1000);
//  HAL_Delay(2000);
    
}

