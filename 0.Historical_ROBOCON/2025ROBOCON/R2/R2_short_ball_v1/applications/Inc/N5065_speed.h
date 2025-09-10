#ifndef __N5065_SPEED_H_
#define __N5065_SPEED_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "struct_typedef.h"
#include "tim.h"

#define ALPHA 0.5

	
void startup_main(void);
	
typedef struct 
{
    fp32 speed_rpm;
    fp32 speed_filtered;
    uint32_t cap_val;
    uint32_t last_cap_val;

    uint32_t cap_now;
    uint32_t last_cap_tim;
    uint8_t cap_init;
    uint32_t Channel;
}motor_measure;


void speed_rpm_calc(TIM_HandleTypeDef *htim, motor_measure *ptr);
void Judge_Motor_Stop(motor_measure *ptr);
float low_pass_filter(float new_speed);
void Motor_PWM_Init(TIM_HandleTypeDef *htim);
#ifdef __cplusplus
}
#endif
	
#endif